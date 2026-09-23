/**
 * @file    test_desc_cache.cpp
 * @brief   描述符缓存（ADR-10 路线 B）与 §10.2 ⑥「下载被中断 → 缓存不落盘」
 *
 * @par 为什么这条必须用**真故障注入**来测
 *  "缓存不落盘"是个守卫，守卫最怕的就是"看起来在、其实没生效"。
 *  虚拟后端提供了 `jsdk_hal_virtual_set_tx_fail()`，所以我们能真的让描述符下载失败，
 *  然后断言**缓存目录里没有文件** —— 而不是"看代码相信它写了 `if (complete)`"。
 */

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "jr_ros2/rt/jr_bus_runtime.hpp"
#include "jr_ros2/rt/jr_desc_cache.hpp"
#include "jr_test.hpp"

/* 虚拟后端的故障注入（C API；测试允许直接用它）。
   ⚠ 必须在 `extern "C"` 里 include：SDK 是 C 库，C++ 里不包会链接失败
     （名字修饰不一致 —— 本项目已踩过一次同类坑）。 */
extern "C" {
#include "joint_sdk/joint_sdk.h"
#include "joint_sdk/jsdk_hal_builtin.h"
}

using namespace jr;
using namespace jr::rt;

namespace {

bool file_exists(const char *path)
{
    std::FILE *f = std::fopen(path, "rb");
    if (f == nullptr) return false;
    std::fclose(f);
    return true;
}

void remove_dir_files(const char *dir, const char *key)
{
    DescCache c;
    char msg[128] = {};
    (void)c.remove(dir, key, msg, sizeof(msg));
    char path[kPathLen * 2] = {};
    DescCache::path_of(dir, key, path, sizeof(path));
    std::remove(path);
}

/** 一份最小虚拟总线配置（描述符缓存打开，锁关掉——测试不该抢系统锁）。 */
Config make_cfg(const char *cache_dir, bool cache_enabled)
{
    Config c = {};
    c.bus_count = 1u;
    BusCfg &b = c.buses[0];
    std::snprintf(b.name, sizeof(b.name), "%s", "vbus");
    b.hal = HalKind::kVirtual;
    std::snprintf(b.channel, sizeof(b.channel), "%s",
                  "0:id=1,gear=16.5,pmax=12.5,vmax=65,tmax=50,hb=5,fd");
    b.is_fd = true;
    b.master_id = 1u;
    b.joint_count = 1u;
    std::snprintf(b.joints[0].name, sizeof(b.joints[0].name), "%s", "j0");
    b.joints[0].node_id = 1u;
    b.desc.cache_enabled = cache_enabled;
    b.desc.retries = 1u;             /* 测试里别等太久 */
    b.desc.retry_backoff_ms = 20u;
    if (cache_dir != nullptr) std::snprintf(b.desc.cache_dir, sizeof(b.desc.cache_dir), "%s", cache_dir);
    std::snprintf(c.groups[0].name, sizeof(c.groups[0].name), "%s", "g0");
    c.groups[0].rate_hz = 1000u;
    c.groups[0].bus_index[0] = 0u;
    c.groups[0].bus_count = 1u;
    c.group_count = 1u;
    c.lock.enabled = false;
    return c;
}

/* ==========================================================================
 * ① 缓存契约：往返 / 失效键 / 损坏不得静默接受
 * ======================================================================== */

void test_cache_contract()
{
    JR_CASE("缓存往返、fw 失效键、损坏数据必须被拒绝（路线 B 的原始 JSON）");

    const char *dir = "jr_test_cache_unit/nested";   /* 多级且**不存在**：顺便验"自动建目录" */
    const char *json = "{\"ep\":[{\"p\":\"axis0.motor.config.gear_ratio\"}]}";
    const std::size_t len = std::strlen(json);
    char msg[192] = {};
    DescCache cache;
    remove_dir_files(dir, "vbus");

    /* 写入：目录不存在时必须**自动创建**（否则缓存永远不生效） */
    JR_CHECK_MSG(cache.save(dir, "vbus", 0x0102u, 0x1234u, json, len, msg, sizeof(msg)) ==
                     CacheOutcome::kHit,
                 msg);

    /* 命中：内容与元数据都要对得上 */
    std::vector<std::uint8_t> out;
    DescCacheMeta meta;
    JR_CHECK(cache.load(dir, "vbus", 0x0102u, &out, &meta, msg, sizeof(msg)) == CacheOutcome::kHit);
    /* ⚠ 先看长度再比较：加载失败时 `out` 是空的，直接 memcmp 会读越界崩溃
       （第一版就这么崩了 —— 断言宁可"错"也不能崩）。 */
    JR_CHECK_EQ(out.size(), len);
    if (out.size() == len) {
        JR_CHECK_EQ(std::memcmp(out.data(), json, len), 0);
    }
    JR_CHECK_EQ(meta.fw_version, 0x0102u);
    JR_CHECK_EQ(meta.crc, 0x1234u);

    /* 失效键：设备 fw 变了 → 必须 miss（而不是拿旧缓存启动） */
    out.clear();
    JR_CHECK(cache.load(dir, "vbus", 0x9999u, &out, &meta, msg, sizeof(msg)) != CacheOutcome::kHit);
    JR_CHECK_EQ(out.size(), 0u);

    /* 损坏：改掉 payload 里一个字节 → 绝不允许当命中 */
    char path[kPathLen * 2] = {};
    DescCache::path_of(dir, "vbus", path, sizeof(path));
    {
        std::FILE *f = std::fopen(path, "r+b");
        JR_CHECK(f != nullptr);
        if (f != nullptr) {
            /* 文件尾部往前一点是 payload（头部有 magic/版本/fw/crc/长度） */
            std::fseek(f, 0, SEEK_END);
            const long sz = std::ftell(f);
            if (sz > 8) {
                std::fseek(f, sz - 4, SEEK_SET);
                const char flipped = 0x5A;
                std::fwrite(&flipped, 1u, 1u, f);
            }
            std::fclose(f);
        }
    }
    out.clear();
    JR_CHECK_MSG(cache.load(dir, "vbus", 0x0102u, &out, &meta, msg, sizeof(msg)) != CacheOutcome::kHit,
                 "损坏的缓存必须被拒绝（否则下次上电就用一份坏缓存启动）");

    remove_dir_files(dir, "vbus");
}

/* ==========================================================================
 * ② §10.2 ⑥：下载被中断 → 缓存不落盘（真故障注入）
 * ======================================================================== */

void test_interrupted_download_writes_no_cache()
{
    JR_CASE("描述符下载失败时**绝不**落缓存（§10.2 ⑥，真注入 TX 失败）");

    const char *good_dir = "jr_test_cache_good";
    const char *bad_dir = "jr_test_cache_bad";
    remove_dir_files(good_dir, "vbus");
    remove_dir_files(bad_dir, "vbus");

    OpenOptions opt;
    opt.enable_lock = false;
    opt.check_abi = false;   /* 这一条测的是缓存，不是 ABI */

    /* ---- ① 正常一次：应当落缓存，且第二次能从缓存加载 ---- */
    {
        Config cfg = make_cfg(good_dir, true);
        BusRuntime rt_bus;
        Result r;
        BusReport rep;
        JR_CHECK_MSG(rt_bus.open(cfg.buses[0], 1000000u, opt, &r) == Status::kOk, r.message);
        JR_CHECK_MSG(rt_bus.configure(&rep, &r) == Status::kOk, r.message);
        JR_CHECK_EQ(rep.desc.complete, true);
        JR_CHECK_EQ(rep.desc.from_cache, false);
        rt_bus.close(ExitAction::kDisable, nullptr);

        char path[kPathLen * 2] = {};
        DescCache::path_of(good_dir, "vbus", path, sizeof(path));
        JR_CHECK_MSG(file_exists(path), "成功下载后缓存文件必须存在");

        /* 第二次：应该 from_cache 命中（这就是缓存的价值） */
        BusRuntime rt2;
        BusReport rep2;
        JR_CHECK_MSG(rt2.open(cfg.buses[0], 1000000u, opt, &r) == Status::kOk, r.message);
        JR_CHECK_MSG(rt2.configure(&rep2, &r) == Status::kOk, r.message);
        /* 第二次：**不该再下载**。缓存有两层：
             - SDK 的**进程内**描述符缓存（同一个进程里第二次 configure 会命中它 → `shared_hit`）；
             - 我们的**文件**缓存（跨进程/重启才用得上 → `from_cache`）。
           所以这里判"命中任一层"（跨进程那一层由 test ③ 的 DescCache 往返覆盖）。 */
        JR_CHECK_MSG(rep2.desc.from_cache || rep2.desc.shared_hit,
                     "第二次 configure 不该重新下载描述符（进程内命中或文件缓存命中皆可）");
        JR_CHECK_EQ(rep2.desc.complete, true);
        rt2.close(ExitAction::kDisable, nullptr);
    }

    /* ---- ② 坏的一次：TX 全失败 → 下载失败 → 缓存目录必须**空** ---- */
    {
        Config cfg = make_cfg(bad_dir, true);
        BusRuntime rt_bus;
        Result r;
        BusReport rep;
        JR_CHECK_MSG(rt_bus.open(cfg.buses[0], 1000000u, opt, &r) == Status::kOk, r.message);

        auto *hal = static_cast<jsdk_hal_handle_t *>(rt_bus.hal_handle_for_test());
        JR_CHECK(hal != nullptr);
        jsdk_hal_virtual_set_tx_fail(hal, -1);   /* 后续发送全部失败 */

        const Status st = rt_bus.configure(&rep, &r);
        JR_CHECK_MSG(st != Status::kOk, "TX 全失败时 configure 必须失败，不能假装成功");
        JR_CHECK_MSG(!rep.desc.complete, "描述符不该是 complete（下载没跑完）");
        rt_bus.close(ExitAction::kDisable, nullptr);
    }
    {
        char path[kPathLen * 2] = {};
        DescCache::path_of(bad_dir, "vbus", path, sizeof(path));
        JR_CHECK_MSG(!file_exists(path),
                     "★ 下载被中断时**不得**落缓存（否则下次上电会用一份坏缓存启动）");
    }

    remove_dir_files(good_dir, "vbus");
    remove_dir_files(bad_dir, "vbus");
}

/* ==========================================================================
 * ③ 默认目录 / 路径拼装（诊断与产线工具都要用）
 * ======================================================================== */

void test_paths()
{
    JR_CASE("默认缓存目录与路径拼装（诊断里要显示它）");

    char dir[kPathLen] = {};
    DescCache::default_dir(dir, sizeof(dir));
    JR_CHECK(dir[0] != '\0');
    JR_CHECK_MSG(std::strstr(dir, "jr") != nullptr, "默认目录里应当带 'jr' 以便现场辨认");

    char path[kPathLen * 2] = {};
    DescCache::path_of("/tmp/jr-cache-test", "can0", path, sizeof(path));
    JR_CHECK_MSG(std::strstr(path, "can0") != nullptr, "路径里必须能看出是哪条总线的缓存");
}

}  // namespace

int main()
{
    test_cache_contract();
    test_interrupted_download_writes_no_cache();
    test_paths();
    return jrtest::report();
}
