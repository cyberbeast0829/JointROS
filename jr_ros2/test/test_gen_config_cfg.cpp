/**
 * @file    test_gen_config_cfg.cpp
 * @brief   `jr_gen_config` 的“命令行 → 扫描用总线配置”映射
 *
 * @par 这组断言守的是两个**只有真机才能暴露**的 bug（DESIGN §13.3-47）
 *  ① slcan 漏 `serial_baud` ⇒ 生成的 YAML **连我们自己的加载器都过不了**
 *     （「生成→回读」自检当场拒绝落盘；**自检本身没错，错的是那段代码路径只有真机才到得了**
 *      —— CI 跑的是 `--if virtual`，而 virtual 不需要 serial_baud）；
 *  ② `--is-fd` 没进总线配置 ⇒ 在 Classic 设备上按 FD 扫描，报出来的是毫无线索的
 *     `descriptor download stalled (0/0 bytes)`。
 *
 * @par 为什么断言要落在 `validate_config()` 上
 *  它是配置的**唯一闸门**（“能不能真的用”由它说了算），比逐字段对拍更强：
 *  以后谁给某个后端加了新的必填字段，这里会立刻红，而不用等真机。
 */

#include <cstdio>
#include <cstring>
#include <string>

#include "jr_ros2/jr_config.hpp"
#include "jr_ros2/jr_gen_config_cfg.hpp"
#include "jr_test.hpp"

using namespace jr;
using jr::tools::build_scan_bus_cfg;
using jr::tools::ScanCfgOpts;

namespace {

/** 把「扫描用总线配置」补成一份**完整可校验**的配置后过一遍**真闸门**（validate_config）。
    扫描时还不知道总线上有几台设备、生成器也还没写 tick_groups，而闸门要求
    “每条总线恰好被一个 tick 组驱动” + “至少一台关节” ⇒ 这里补齐这两样。 */
Result validate_scan_bus(const BusCfg &scan_cfg)
{
    Config c = default_config();
    c.bus_count = 1u;
    c.buses[0] = scan_cfg;
    BusCfg &b = c.buses[0];
    b.joint_count = 1u;
    std::snprintf(b.joints[0].name, sizeof b.joints[0].name, "%s", "j1");
    b.joints[0].node_id = 1u;

    c.group_count = 1u;
    TickGroupCfg &g = c.groups[0];
    std::snprintf(g.name, sizeof g.name, "%s", "g0");
    g.rate_hz = 1000u;
    g.bus_count = 1u;
    g.bus_index[0] = 0u;

    ConfigNotes notes;
    return validate_config(c, &notes);
}

void test_slcan_needs_serial_baud()
{
    JR_CASE("① slcan 必须带 serial_baud（否则生成的配置自己都加载不了）");

    ScanCfgOpts o;
    o.channel = "/dev/ttyACM0";
    o.bus_name = "axis1";
    o.is_fd = 0;
    const BusCfg bc = build_scan_bus_cfg(HalKind::kSlcan, o);

    /* 字段级：**串口**速率必须落到 BusCfg 上（默认 115200，可覆盖）。 */
    JR_CHECK_EQ(static_cast<unsigned>(bc.serial_baud), 115200u);
    JR_CHECK(bc.hal == HalKind::kSlcan);
    JR_CHECK(std::strcmp(bc.channel, "/dev/ttyACM0") == 0);
    JR_CHECK(std::strcmp(bc.name, "axis1") == 0);

    ScanCfgOpts o2 = o;
    o2.serial_baud = 921600u;
    JR_CHECK_EQ(static_cast<unsigned>(build_scan_bus_cfg(HalKind::kSlcan, o2).serial_baud), 921600u);

    /* 闸门级（这条才是真正的守卫）：slcan 的扫描配置必须能过 validate_config。 */
    const Result r = validate_scan_bus(bc);
    JR_CHECK_MSG(r.ok(), r.message);
    if (!r.ok()) std::printf("  slcan 扫描配置未过闸门：%s\n", r.message);

    /* **反向断言**：把 serial_baud 抹掉，闸门必须**拒绍** —— 证明上面那条不是空转
       （否则“过了闸门”可能只是因为 validate_config 对 slcan 什么都不查）。 */
    BusCfg broken = bc;
    broken.serial_baud = 0u;
    const Result bad = validate_scan_bus(broken);
    JR_CHECK_MSG(!bad.ok(), "validate_config 竟然放过了 serial_baud=0 的 slcan 总线");
}

void test_is_fd_reaches_the_bus()
{
    JR_CASE("② --is-fd 必须真的落到扫描用的总线配置上");

    ScanCfgOpts o;
    o.channel = "/dev/ttyACM0";
    o.bus_name = "axis1";

    /* 明确指定 classic / FD */
    o.is_fd = 0;
    JR_CHECK(build_scan_bus_cfg(HalKind::kSlcan, o).is_fd == false);
    o.is_fd = 1;
    JR_CHECK(build_scan_bus_cfg(HalKind::kSlcan, o).is_fd == true);

    /* 没给（-1）→ **Classic 起步**（SDK 推荐组合：FD 控制器也收经典帧，反之不成立；
       SDK 会按对端第一帧自动对齐并如实报告）。这条同时是"默认值语义"的守卫。 */
    o.is_fd = -1;
    JR_CHECK_MSG(build_scan_bus_cfg(HalKind::kSlcan, o).is_fd == false,
                 "未指定 --is-fd 时应当 Classic 起步（SDK 推荐），而不是默认 FD");}

void test_every_backend_builds_a_valid_bus()
{
    JR_CASE("每个后端都要能造出一条能过闸门的总线（新后端别忘字段）");

    struct Case {
        HalKind     hal;
        const char *channel;
        const char *bus_name;
        unsigned    serial_baud;
    };
    const Case cases[] = {
        {HalKind::kSocketCan, "can0", "can0", 0u},
        {HalKind::kSlcan, "/dev/ttyACM0", "axis1", 115200u},
        {HalKind::kPcan, "PCAN_USBBUS1", "pcan1", 0u},
        {HalKind::kVirtual, "0:id=1,gear=7.75,pmax=12.5,vmax=65,tmax=50,hb=100", "virt", 0u},
    };
    for (const Case &cs : cases) {
        ScanCfgOpts o;
        o.channel = cs.channel;
        o.bus_name = cs.bus_name;
        o.serial_baud = cs.serial_baud;
        const BusCfg bc = build_scan_bus_cfg(cs.hal, o);
        const Result r = validate_scan_bus(bc);
        if (!r.ok()) {
            std::printf("  [%s] 扫描配置未过闸门：%s\n", to_string(cs.hal), r.message);
        }
        JR_CHECK(r.ok());
    }
}

}  // namespace

int main()
{
    std::printf("== jr_gen_config 的“命令行 → 总线配置”映射 ==\n");
    test_slcan_needs_serial_baud();
    test_is_fd_reaches_the_bus();
    test_every_backend_builds_a_valid_bus();
    return ::jrtest::report();
}
