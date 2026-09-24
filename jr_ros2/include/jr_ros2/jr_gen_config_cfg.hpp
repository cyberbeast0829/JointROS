// ============================================================================
//  jr_gen_config_cfg —— `jr_gen_config` 的“命令行 → 扫描用总线配置”映射
// ----------------------------------------------------------------------------
//  ⚠ 为什么单独抽成**纯函数**（两个只有真机能发现的 bug，DESIGN §13.3-47）：
//    ① slcan 忘了 `serial_baud` ⇒ 生成的 YAML **连我们自己的加载器都过不了**
//       （加载器的“生成→回读”自检当场拒绝落盘 —— 自检有效，但那段代码路径只有在真机上
//       跑完扫描才会到达，CI 里跑的是 `--if virtual`，virtual 不需要 serial_baud）；
//    ② `--is-fd` 只影响生成文件里的**文本**，不影响**扫描时开总线的参数** ⇒ 在 Classic
//       设备上按 FD 扫描，必然失败，而报出来的是一句毫无线索的 `0/0 bytes`。
//    两个都属于“命令行选项 → 总线配置”这段映射。藏在 `main()` 里，就只能靠真机暴露；
//    收进一处 + 用 `validate_config()` 断言（见 `test/test_gen_config_cfg.cpp`），离线就能守住。
// ============================================================================
#pragma once

#include <cstdint>
#include <cstdio>
#include <string>

#include "jr_ros2/jr_config.hpp"

namespace jr {
namespace tools {

/** `jr_gen_config` 里与“开总线”有关的那部分命令行选项。 */
struct ScanCfgOpts {
    std::string channel;               /**< socketcan/pcan/slcan = 接口名；virtual = 规格串 */
    std::string bus_name;              /**< 空 = 取 channel（virtual 取 "virt"） */
    unsigned    master_id = 1u;
    /** -1 = 命令行没给 ⇒ **Classic 起步**（SDK 会按对端帧自动对齐）；
        0 / 1 = 明确指定。⚠ 以前这个值压根没进总线配置（§13.3-47 ②）。 */
    int         is_fd = -1;
    unsigned    serial_baud = 115200u; /**< 仅 slcan：**串口**速率，不是 CAN 速率 */
};

/** 构造“扫描用”的总线配置。**每个后端需要哪些字段只有这里一处**，别在 main() 里重写。 */
inline BusCfg build_scan_bus_cfg(HalKind hal, const ScanCfgOpts &o)
{
    BusCfg bc = default_bus_cfg();
    const std::string name =
        o.bus_name.empty() ? (hal == HalKind::kVirtual ? std::string("virt") : o.channel) : o.bus_name;
    std::snprintf(bc.name, sizeof bc.name, "%s", name.c_str());
    bc.hal = hal;
    std::snprintf(bc.channel, sizeof bc.channel, "%s", o.channel.c_str());
    bc.master_id = static_cast<std::uint8_t>(o.master_id);
    bc.joint_count = 0u;   /* 扫描前还不知道有几台设备 */

    /* ⚠ slcan 的 schema 校验要求 `serial_baud`（"slcan requires serial_baud (e.g. 115200);
       it is the SERIAL rate, not the CAN rate"）。漏了它，生成的配置连自己的加载器都过不了。 */
    if (hal == HalKind::kSlcan) bc.serial_baud = o.serial_baud;

    /* `--is-fd` 必须真的落到“开总线”的参数上（§13.3-47 ②）：
       没给时 Classic 起步 —— FD 控制器**也收**经典帧、反之不成立（SDK 的推荐组合）。 */
    bc.is_fd = (o.is_fd < 0) ? false : (o.is_fd != 0);
    return bc;
}

}  // namespace tools
}  // namespace jr
