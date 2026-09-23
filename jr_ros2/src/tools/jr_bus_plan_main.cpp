/**
 * @file    jr_bus_plan_main.cpp
 * @brief   `jr_bus_plan` —— **先算再上机**：把配置代进 §7.1 的总线预算模型
 *
 * @par 为什么要一个独立工具（模型已经在核心库里了）
 *  预算模型 `plan_bus()` 已被节点/ros2_control/`jr_hw_verify`/`jr_gen_config` 共用，
 *  但它只在"启动"或"体检"时才跑 —— 而客户真正需要它的时刻是**画线之前**：
 *  "这条线上放几个关节、开不开 FD、反馈策略选哪种、控制率多少"。
 *  这个 CLI 把那一步变成一条**不需要接硬件**的命令（纯读配置 + 算账）。
 *
 * @par 它不做的事（如实说）
 *  不打开总线、不读设备 → 用的是**配置里写的**量程/波特率（配置错了它算出来的也就不对）。
 *  真要核对设备侧真值，用 `jr_hw_verify`。
 *
 * @par 退出码：0 = 全部总线可行；1 = 有不可行项（逐条给出建议）；2 = 用法/配置错误
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "jr_ros2/jr_config.hpp"
#include "jr_ros2/jr_config_yaml.hpp"
#include "jr_ros2/rt/jr_bus_plan.hpp"

namespace {

constexpr int kExitOk = 0;
constexpr int kExitInfeasible = 1;
constexpr int kExitUsage = 2;

void usage()
{
    std::fputs(
        "jr_bus_plan -- 总线预算检查（先算再上机；不接硬件、不打开总线）\n"
        "\n"
        "用法：\n"
        "  jr_bus_plan --config <file> [--rate-hz N] [--quiet]\n"
        "\n"
        "选项：\n"
        "  --config <file>   被检查的配置（DESIGN §6.4 的 YAML）\n"
        "  --rate-hz <N>     控制率（默认取配置里第一个 tick 组的 rate_hz）\n"
        "  --quiet           只打印不可行项与汇总\n"
        "  -h, --help        本帮助\n"
        "\n"
        "判据（DESIGN §7.1）：关节数 × 帧数 × 帧时间 ≤ tick 预算的 60%；\n"
        "node_id > 7 的关节无法参与广播同步（会被算成逐关节单播）。\n"
        "\n"
        "⚠ 它用的是**配置里写的**参数。设备的真实量程/波特率以 `jr_hw_verify` 的读回值为准。\n",
        stdout);
}

}  // namespace

int main(int argc, char **argv)
{
    std::string cfg_path;
    double rate_hz = 0.0;   /* 0 = 用配置里的 */
    bool quiet = false;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        const auto value = [&](const char *what) -> const char * {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "jr_bus_plan: %s 需要一个值\n", what);
                return nullptr;
            }
            return argv[++i];
        };
        if (a == "-h" || a == "--help") { usage(); return kExitOk; }
        else if (a == "--config") { const char *v = value("--config"); if (!v) return kExitUsage; cfg_path = v; }
        else if (a == "--quiet") { quiet = true; }
        else if (a == "--rate-hz") {
            const char *v = value("--rate-hz"); if (!v) return kExitUsage;
            char *end = nullptr; const double d = std::strtod(v, &end);
            if (end == nullptr || *end != '\0' || !(d >= 1.0 && d <= 20000.0)) {
                std::fprintf(stderr, "jr_bus_plan: --rate-hz 需要 1..20000（得到 '%s'）\n", v);
                return kExitUsage;
            }
            rate_hz = d;
        } else {
            std::fprintf(stderr, "jr_bus_plan: 未知选项 '%s'（--help 看用法）\n", a.c_str());
            return kExitUsage;
        }
    }
    if (cfg_path.empty()) {
        std::fputs("jr_bus_plan: 缺少 --config（--help 看用法）\n", stderr);
        return kExitUsage;
    }

    jr::Config cfg;
    jr::Result res;
    jr::YamlLoadReport rep;
    const jr::Status st = jr::load_config_yaml(cfg_path.c_str(), &cfg, &res, &rep);
    if (st != jr::Status::kOk) {
        std::fprintf(stderr, "jr_bus_plan: 配置加载失败：%s (%s)\n", res.message, jr::to_string(st));
        return kExitUsage;
    }
    if (rep.notes.text[0] != '\0' && !quiet) {
        std::printf("配置备注：%s\n", rep.notes.text);
    }

    /* 控制率：命令行优先；否则用配置里第一个 tick 组（客户就是这么描述自己系统的）。 */
    if (rate_hz <= 0.0) {
        rate_hz = (cfg.group_count > 0u) ? static_cast<double>(cfg.groups[0].rate_hz) : 1000.0;
    }
    std::printf("jr_bus_plan: %u 条总线，控制率 %.0f Hz%s\n", cfg.bus_count, rate_hz,
                (cfg.command.timeout_ms == 0u) ? "（⚠ command.timeout_ms=0：命令超时闸已关闭）" : "");

    unsigned infeasible = 0u;
    for (unsigned b = 0u; b < cfg.bus_count; ++b) {
        const jr::BusCfg &bus = cfg.buses[b];
        const jr::rt::BusPlanResult plan = jr::rt::plan_bus(bus, rate_hz);
        std::printf("\n[总线 %u/%u] '%s'（%s，%u 关节，%s）\n", b + 1u, cfg.bus_count, bus.name,
                    jr::to_string(bus.hal), bus.joint_count, bus.is_fd ? "CAN-FD" : "Classic");
        if (!plan.feasible) {
            ++infeasible;
            std::printf("  [FAIL] 不可行：%s\n", plan.text);
        } else if (!quiet) {
            std::printf("  [ok]   %s\n", plan.text);
        }
        if (plan.warn) {
            /* 模型自己判的"接近上限"：仍然可行，但余量不足以应付重试/错误帧 —— 说出来。 */
            std::printf("  [warn] 负载偏高（%.1f%%）：重试/错误帧的余量很小\n", plan.load * 100.0);
        }
        if (!quiet) {
            /* 打印**原始数字**而不是只给一句结论：客户要拿它跟自己手算/别的工具对拍。 */
            std::printf("  [info] 负载 %.1f%%（上限 %.0f%%）、TX %.0f 帧/s、RX %.0f 帧/s、"
                        "每 tick 在网 %.1f µs、单播帧 %.1f µs、广播帧 %.1f µs\n",
                        plan.load * 100.0, bus.max_bus_load * 100.0, plan.tx_frames_per_sec,
                        plan.rx_frames_per_sec, plan.us_per_tick, plan.frame_us_unicast,
                        plan.frame_us_broadcast);
        }
        /* node_id > 7：协议硬限制（广播位图只到 7）—— 这条最容易被忽略，单独点名。 */
        for (unsigned j = 0u; j < bus.joint_count; ++j) {
            if (bus.joints[j].node_id > jr::kMaxBroadcastNodeId) {
                std::printf("  [warn] 关节 '%s' 的 node_id=%u > %u ⇒ **无法参与广播同步**"
                            "（会被逐关节单播，帧数上升）\n",
                            bus.joints[j].name, static_cast<unsigned>(bus.joints[j].node_id),
                            static_cast<unsigned>(jr::kMaxBroadcastNodeId));
            }
        }
    }

    std::printf("\n汇总：%u 条总线，%u 条不可行\n", cfg.bus_count, infeasible);
    return (infeasible == 0u) ? kExitOk : kExitInfeasible;
}
