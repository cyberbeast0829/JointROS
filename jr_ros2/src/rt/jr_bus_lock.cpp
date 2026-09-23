/**
 * @file    jr_bus_lock.cpp
 * @brief   总线排他锁实现（POSIX flock / Windows LockFileEx）
 */

#include "jr_ros2/rt/jr_bus_lock.hpp"

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <cstdlib>

#if defined(_WIN32)
#  include <io.h>
#  include <windows.h>
#else
#  include <cerrno>
#  include <fcntl.h>
#  include <sys/file.h>
#  include <sys/stat.h>
#  include <sys/types.h>
#  include <unistd.h>
#endif

namespace jr {
namespace {

void sanitize(const char *in, char *out, std::size_t cap) noexcept
{
    std::size_t o = 0u;
    for (std::size_t i = 0u; in != nullptr && in[i] != '\0' && o + 1u < cap; ++i) {
        const char c = in[i];
        const bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                        (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.';
        out[o++] = ok ? c : '_';
    }
    out[o] = '\0';
}

/** 解析默认锁目录（调用方给的 dir 为空时使用）。 */
void resolve_dir(const char *dir, char *out, std::size_t cap) noexcept
{
    if (dir != nullptr && dir[0] != '\0') {
        std::snprintf(out, cap, "%s", dir);
        return;
    }
#if defined(_WIN32)
    const char *base = std::getenv("LOCALAPPDATA");
    if (base == nullptr || base[0] == '\0') base = std::getenv("TEMP");
    if (base == nullptr || base[0] == '\0') base = ".";
    std::snprintf(out, cap, "%s", base);
#else
    std::snprintf(out, cap, "%s", "/var/lock");
#endif
}

/**
 * 读锁文件里的持有人 PID（写成 "pid N"）。
 *
 * ⚠ Windows 下必须锁在**别的区间**（见 acquire 里的说明），否则这一段永远读不到。
 */
std::size_t read_pid_text(const char *path, char *out, std::size_t cap) noexcept
{
    if (out == nullptr || cap == 0u) return 0u;
    out[0] = '\0';
    std::FILE *f = std::fopen(path, "rb");
    if (f == nullptr) return 0u;
    long pid = -1;
    /* ⚠ 格式串必须与写入端一致（写的是 "pid N"）。
       踩过：写入端 "pid %lu"、读取端 "%ld" → fscanf 在 'p' 上匹配失败返回 0，
       症状是"锁被占着但永远显示未知持有人"。 */
    const bool got = (std::fscanf(f, "pid %ld", &pid) == 1);
    std::fclose(f);
    if (!got) return 0u;
    const int n = std::snprintf(out, cap, "pid %ld", pid);
    return (n > 0) ? static_cast<std::size_t>(n) : 0u;
}

}  // namespace

BusLock::~BusLock() { release(); }

int BusLock::owner_pid_on_disk() const noexcept
{
    if (owner_path_[0] == '\0' && path_[0] == '\0') return -1;
    char text[64] = {};
    const char *p = (owner_path_[0] != '\0') ? owner_path_ : path_;
    if (read_pid_text(p, text, sizeof(text)) == 0u) return -1;
    long pid = -1;
    if (std::sscanf(text, "pid %ld", &pid) != 1) return -1;
    return static_cast<int>(pid);
}

Status BusLock::acquire(const char *bus_name, const char *lock_dir, bool allow_shared, char *msg,
                        std::size_t msg_len) noexcept
{
    if (held_) return Status::kOk;
    if (bus_name == nullptr || bus_name[0] == '\0') {
        if (msg != nullptr && msg_len > 0u) std::snprintf(msg, msg_len, "bus_lock: empty bus name");
        return Status::kInvalidArgument;
    }

    char dir[256] = {};
    char safe[96] = {};
    resolve_dir(lock_dir, dir, sizeof(dir));
    sanitize(bus_name, safe, sizeof(safe));
    std::snprintf(path_, sizeof(path_), "%s%cjr-%s.lock", dir,
#if defined(_WIN32)
                  '\\',
#else
                  '/',
#endif
                  safe);
    /* PID 写到**不加锁**的伴随文件：
       ⚠ Windows 实测（probe）：排他字节范围锁持有期间，**其它句柄连锁区间之外的字节
          也读不到**（fscanf 返回 -1）。所以把 PID 写在加锁文件里，
          "锁被占着时告诉用户谁占着"这个诊断就无法实现。分离文件后两端都成立。 */
    std::snprintf(owner_path_, sizeof(owner_path_), "%s%cjr-%s.owner", dir,
#if defined(_WIN32)
                  '\\',
#else
                  '/',
#endif
                  safe);

    shared_mode_ = allow_shared;

#if defined(_WIN32)
    HANDLE h = CreateFileA(path_, GENERIC_READ | GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE,
                           nullptr, OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (h == INVALID_HANDLE_VALUE) {
        if (msg != nullptr && msg_len > 0u) {
            std::snprintf(msg, msg_len, "bus_lock: cannot open '%s' (err=%lu)", path_,
                          static_cast<unsigned long>(GetLastError()));
        }
        return Status::kIoError;
    }
    if (!allow_shared) {
        OVERLAPPED ov;
        std::memset(&ov, 0, sizeof(ov));
        if (LockFileEx(h, LOCKFILE_EXCLUSIVE_LOCK | LOCKFILE_FAIL_IMMEDIATELY, 0, 1u, 0u, &ov) == 0) {
            CloseHandle(h);
            if (msg != nullptr && msg_len > 0u) {
                char owner[64] = {};
                if (read_pid_text(owner_path_, owner, sizeof(owner)) == 0u) {
                    std::snprintf(owner, sizeof(owner), "%s", "an unknown process");
                }
                std::snprintf(msg, msg_len,
                              "bus_lock: '%s' is held by %s. Stop the other master (e.g. jsdk-cli), or set "
                              "lock.allow_shared=true if you really need two (two masters on one "
                              "CAN make devices remember the LAST master id: interleaved "
                              "heartbeats, flaky parameter writes).",
                              path_, owner);
            }
            return Status::kLocked;
        }
    }
    /* ⚠ 必须保存句柄：`release()` 靠它解锁 —— 丢过一行 `handle_ = h`，
       症状是“进程退出前锁永远不释放”（测试里表现为 release 后别人拿不到锁）。 */
    handle_ = reinterpret_cast<void *>(h);
#else
    const int fd = ::open(path_, O_RDWR | O_CREAT, 0644);
    if (fd < 0) {
        if (msg != nullptr && msg_len > 0u) {
            std::snprintf(msg, msg_len, "bus_lock: cannot open '%s' (%s)", path_, std::strerror(errno));
        }
        return Status::kIoError;
    }
    if (!allow_shared) {
        if (::flock(fd, LOCK_EX | LOCK_NB) != 0) {
            char owner[64] = {};
            if (read_pid_text(owner_path_, owner, sizeof(owner)) == 0u) {
                std::snprintf(owner, sizeof(owner), "%s", "an unknown process");
            }
            ::close(fd);
            if (msg != nullptr && msg_len > 0u) {
                std::snprintf(msg, msg_len,
                              "bus_lock: '%s' is held by %s. Stop the other master (e.g. jsdk-cli), or set "
                              "lock.allow_shared=true if you really need two (two masters on one "
                              "CAN make devices remember the LAST master id: interleaved "
                              "heartbeats, flaky parameter writes).",
                              path_, owner);
            }
            return Status::kLocked;
        }
    }
    handle_ = reinterpret_cast<void *>(static_cast<std::intptr_t>(fd + 1));
#endif

    /* 把本进程 PID 写进**不加锁**的伴随文件（两个平台共用同一条路径）。
       写失败只降级为"诊断不完整"，不影响锁的正确性。 */
    {
        std::FILE *of = std::fopen(owner_path_, "wb");
        if (of != nullptr) {
#if defined(_WIN32)
            const unsigned long pid = static_cast<unsigned long>(GetCurrentProcessId());
#else
            const unsigned long pid = static_cast<unsigned long>(::getpid());
#endif
            std::fprintf(of, "pid %lu\n", pid);
            std::fclose(of);
        }
    }

    held_ = true;
    if (msg != nullptr && msg_len > 0u) {
        if (allow_shared) {
            std::snprintf(msg, msg_len, "bus_lock: shared mode enabled (no exclusivity) - %s", path_);
        } else {
            std::snprintf(msg, msg_len, "bus_lock: exclusive on %s", path_);
        }
    }
    return Status::kOk;
}

void BusLock::release() noexcept
{
    if (!held_) {
        path_[0] = '\0';
        return;
    }
    /* 锁文件的 PID 在伴随文件里（见 acquire 说明）。 */
    (void)std::remove(owner_path_);
#if defined(_WIN32)
    if (handle_ != nullptr) {
        HANDLE h = reinterpret_cast<HANDLE>(handle_);
        OVERLAPPED ov;
        std::memset(&ov, 0, sizeof(ov));
        /* 与 acquire 里锁的区间一致（offset 0, 1 byte） */
        UnlockFileEx(h, 0, 1u, 0u, &ov);
        CloseHandle(h);
    }
#else
    if (handle_ != nullptr) {
        const int fd = static_cast<int>(reinterpret_cast<std::intptr_t>(handle_) - 1);
        ::flock(fd, LOCK_UN);
        ::close(fd);
    }
#endif
    handle_ = nullptr;
    held_ = false;
}

}  // namespace jr
