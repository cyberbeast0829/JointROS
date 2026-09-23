/**
 * @file    jr_desc_cache.cpp
 * @brief   描述符缓存实现
 */

#include "jr_ros2/rt/jr_desc_cache.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <system_error>

#include "joint_sdk/joint_sdk.h"

namespace jr {
namespace {

constexpr std::uint32_t kMagic = 0x4A524443u;  /* "JRDC"（LE） */
constexpr std::uint16_t kFormatVersion = 1u;

#pragma pack(push, 1)
struct FileHeader {
    std::uint32_t magic;
    std::uint16_t format_version;
    std::uint16_t crc;          /* 设备侧 VersionCRC（下载时记录） */
    std::uint32_t fw_version;
    std::uint32_t sdk_abi;      /* 记录写缓存时的 SDK ABI，便于诊断 */
    std::uint32_t json_len;
    std::uint32_t payload_crc32;
};
#pragma pack(pop)

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

}  // namespace

void DescCache::default_dir(char *out, std::size_t cap) noexcept
{
    if (out == nullptr || cap == 0u) return;
    const char *base = std::getenv("XDG_CACHE_HOME");
#if defined(_WIN32)
    if (base == nullptr || base[0] == '\0') base = std::getenv("LOCALAPPDATA");
#endif
    if (base == nullptr || base[0] == '\0') {
        const char *home = std::getenv("HOME");
#if defined(_WIN32)
        if (home == nullptr || home[0] == '\0') home = std::getenv("USERPROFILE");
#endif
        if (home == nullptr || home[0] == '\0') home = ".";
        std::snprintf(out, cap, "%s%c.cache", home,
#if defined(_WIN32)
                      '\\'
#else
                      '/'
#endif
        );
    } else {
        std::snprintf(out, cap, "%s", base);
    }
    const std::size_t n = std::strlen(out);
    if (n + 4u < cap) std::snprintf(out + n, cap - n, "%cjr", 
#if defined(_WIN32)
                                    '\\'
#else
                                    '/'
#endif
    );
}

void DescCache::path_of(const char *dir, const char *key, char *out, std::size_t cap) noexcept
{
    char d[256] = {};
    char k[96] = {};
    if (dir == nullptr || dir[0] == '\0') {
        default_dir(d, sizeof(d));
    } else {
        std::snprintf(d, sizeof(d), "%s", dir);
    }
    sanitize(key, k, sizeof(k));
    std::snprintf(out, cap, "%s%c%s.desc",
#if defined(_WIN32)
                  d, '\\', k
#else
                  d, '/', k
#endif
    );
}

std::uint32_t DescCache::crc32(const void *data, std::size_t len) noexcept
{
    static std::uint32_t table[256];
    static bool init = false;
    if (!init) {
        for (std::uint32_t i = 0u; i < 256u; ++i) {
            std::uint32_t c = i;
            for (int b = 0; b < 8; ++b) {
                c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
            }
            table[i] = c;
        }
        init = true;
    }
    const std::uint8_t *p = static_cast<const std::uint8_t *>(data);
    std::uint32_t c = 0xFFFFFFFFu;
    for (std::size_t i = 0u; i < len; ++i) {
        c = table[(c ^ p[i]) & 0xFFu] ^ (c >> 8);
    }
    return c ^ 0xFFFFFFFFu;
}

CacheOutcome DescCache::load(const char *dir, const char *key, std::uint32_t expect_fw,
                             std::vector<std::uint8_t> *json_out, DescCacheMeta *meta_out,
                             char *msg, std::size_t msg_len) const noexcept
{
    char path[384] = {};
    path_of(dir, key, path, sizeof(path));

    std::FILE *f = std::fopen(path, "rb");
    if (f == nullptr) {
        if (msg != nullptr && msg_len > 0u) std::snprintf(msg, msg_len, "cache miss: %s", path);
        return CacheOutcome::kMiss;
    }

    FileHeader h{};
    if (std::fread(&h, sizeof(h), 1u, f) != 1u) {
        std::fclose(f);
        if (msg != nullptr && msg_len > 0u) std::snprintf(msg, msg_len, "cache header unreadable: %s", path);
        return CacheOutcome::kError;
    }
    if (h.magic != kMagic || h.format_version != kFormatVersion) {
        std::fclose(f);
        if (msg != nullptr && msg_len > 0u) {
            std::snprintf(msg, msg_len, "cache header invalid (magic=0x%08X ver=%u): %s",
                          static_cast<unsigned>(h.magic), static_cast<unsigned>(h.format_version),
                          path);
        }
        return CacheOutcome::kError;
    }
    if (expect_fw != 0u && h.fw_version != expect_fw) {
        std::fclose(f);
        if (msg != nullptr && msg_len > 0u) {
            std::snprintf(msg, msg_len,
                          "cache stale: cached fw=%u but device reports fw=%u (%s)",
                          static_cast<unsigned>(h.fw_version), static_cast<unsigned>(expect_fw),
                          path);
        }
        return CacheOutcome::kMiss;
    }

    std::vector<std::uint8_t> buf(h.json_len);
    if (h.json_len != 0u && std::fread(buf.data(), 1u, h.json_len, f) != h.json_len) {
        std::fclose(f);
        if (msg != nullptr && msg_len > 0u) std::snprintf(msg, msg_len, "cache truncated: %s", path);
        return CacheOutcome::kError;
    }
    std::fclose(f);

    if (crc32(buf.data(), buf.size()) != h.payload_crc32) {
        if (msg != nullptr && msg_len > 0u) {
            std::snprintf(msg, msg_len, "cache payload CRC mismatch (file corrupted): %s", path);
        }
        return CacheOutcome::kError;
    }

    if (json_out != nullptr) *json_out = std::move(buf);
    if (meta_out != nullptr) {
        meta_out->fw_version = h.fw_version;
        meta_out->crc = h.crc;
        meta_out->json_len = h.json_len;
    }
    if (msg != nullptr && msg_len > 0u) {
        std::snprintf(msg, msg_len, "cache hit: fw=%u crc=%u len=%u (%s)",
                      static_cast<unsigned>(h.fw_version), static_cast<unsigned>(h.crc),
                      static_cast<unsigned>(h.json_len), path);
    }
    return CacheOutcome::kHit;
}

CacheOutcome DescCache::save(const char *dir, const char *key, std::uint32_t fw_version,
                             std::uint16_t crc, const void *json, std::size_t len, char *msg,
                             std::size_t msg_len) noexcept
{
    if (json == nullptr || len == 0u) {
        if (msg != nullptr && msg_len > 0u) std::snprintf(msg, msg_len, "cache save: empty payload refused");
        return CacheOutcome::kError;
    }

    char path[384] = {};
    path_of(dir, key, path, sizeof(path));

    /* ⚠ 目录不存在就先建：`default_dir()` 指向 `<cache>/jr`，而**新机器上没有这个目录**，
       `fopen` 会直接失败 → 缓存永远不生效，而且症状只是"每次都重新下载"，没人会注意。
       （被 `test_desc_cache` 逼出来的一条真 bug。）
       用 `std::filesystem`（C++17）：多级路径一次建好，且用 error_code 版本不抛异常。 */
    if (dir != nullptr && dir[0] != '\0') {
        std::error_code ec;
        std::filesystem::create_directories(dir, ec);   /* 已存在不算错 */
        if (ec) {
            if (msg != nullptr && msg_len > 0u) {
                std::snprintf(msg, msg_len, "cache save: cannot create directory '%s': %s", dir,
                              ec.message().c_str());
            }
            return CacheOutcome::kError;
        }
    }

    /* 先写临时文件再改名：避免"写一半断电"留下坏缓存（下次上电会判 CRC 不过，
       但那已经是显式的错误而不是静默的半份数据）。 */
    char tmp[400] = {};
    std::snprintf(tmp, sizeof(tmp), "%s.tmp", path);

    std::FILE *f = std::fopen(tmp, "wb");
    if (f == nullptr) {
        if (msg != nullptr && msg_len > 0u) std::snprintf(msg, msg_len, "cache save: cannot open %s", tmp);
        return CacheOutcome::kError;
    }

    FileHeader h{};
    h.magic = kMagic;
    h.format_version = kFormatVersion;
    h.crc = crc;
    h.fw_version = fw_version;
    h.sdk_abi = JSDK_ABI_VERSION_CAN;   /* 写缓存时的 SDK ABI（仅诊断用） */
    h.json_len = static_cast<std::uint32_t>(len);
    h.payload_crc32 = crc32(json, len);

    const bool ok = (std::fwrite(&h, sizeof(h), 1u, f) == 1u) &&
                    (std::fwrite(json, 1u, len, f) == len);
    std::fclose(f);

    if (!ok) {
        std::remove(tmp);
        if (msg != nullptr && msg_len > 0u) std::snprintf(msg, msg_len, "cache save failed: %s", tmp);
        return CacheOutcome::kError;
    }
    std::remove(path);
    if (std::rename(tmp, path) != 0) {
        if (msg != nullptr && msg_len > 0u) std::snprintf(msg, msg_len, "cache rename failed: %s", path);
        return CacheOutcome::kError;
    }
    if (msg != nullptr && msg_len > 0u) {
        std::snprintf(msg, msg_len, "cache saved: fw=%u crc=%u len=%u (%s)",
                      static_cast<unsigned>(fw_version), static_cast<unsigned>(crc),
                      static_cast<unsigned>(len), path);
    }
    return CacheOutcome::kHit;
}

CacheOutcome DescCache::remove(const char *dir, const char *key, char *msg,
                               std::size_t msg_len) noexcept
{
    char path[384] = {};
    path_of(dir, key, path, sizeof(path));
    if (std::remove(path) != 0) {
        if (msg != nullptr && msg_len > 0u) std::snprintf(msg, msg_len, "cache remove: not found (%s)", path);
        return CacheOutcome::kMiss;
    }
    if (msg != nullptr && msg_len > 0u) std::snprintf(msg, msg_len, "cache removed: %s", path);
    return CacheOutcome::kHit;
}

}  // namespace jr
