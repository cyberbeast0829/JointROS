/**
 * @file    jr_hw_verify_main.cpp
 * @brief   `jr_hw_verify` —— 上机前自检（**只读**；不使能、不发控制帧）
 *
 * @par 为什么要它（而不是"跑一次看看")
 *   JointSDK 的 `docs/` 与 `tools/hw_verify.sh` 里记着两条真实教训：
 *   ① slcan 适配器**打开端口后的头几帧会丢**（实测约 1/10 次进程）→ 单跑一次成功
 *      会把 10% 的失败率掩盖掉；
 *   ② 参数值字节序错时 `read` 仍然"成功返回"（数值是垃圾）→ 只看退出码永远发现不了。
 *  所以这个工具按**层次**逐条给结论，且每条结论都能追到原始数据（节点号/帧数/读回值）。
 *
 * @par 它**不做**什么（如实写在 --help 里，别让客户以为覆盖了）
 *   - 不使能关节、不发任何控制帧（不碰电机）→ **不验证闭环**；闭环验证见
 *     `jr_ros2_control/test/jtc_demo/` 与 `jr_ctl`（WP4 后半段，走服务）；
 *   - 默认**不写参数**（写路径自检需要 `--write-probe`，见 `--help`）；
 *   - 不代表"实时性能达标"（那是 `jr_latency_bench`，未实现）。
 *
 * @par 退出码（沿用 SDK `tools/hw_verify.sh` 的约定）
 *   0 = 全部检查通过；1 = 有失败（逐条打印）；2 = 用法/环境问题（如锁被占）。
 */

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "jr_ros2/jr_config.hpp"
#include "jr_ros2/jr_config_yaml.hpp"
#include "jr_ros2/rt/jr_bus_runtime.hpp"
#include "jr_ros2/rt/jr_identify.hpp"

namespace {

constexpr int kExitOk = 0;
constexpr int kExitFail = 1;
constexpr int kExitUsage = 2;

struct Options {
    std::string config;
    unsigned    probe = 16u;
    bool        allow_uncalibrated = false;
    bool        allow_shared = false;
    bool        quiet = false;
    std::string lock_dir;
};

unsigned g_pass = 0u;
unsigned g_warn = 0u;
unsigned g_fail = 0u;
bool     g_quiet = false;   /* 只压 [ok] 行；告警与失败**永远**打（quiet 不该藏问题） */

void pass(const char *fmt, ...) noexcept
{
    ++g_pass;
    if (g_quiet) return;
    va_list ap;
    va_start(ap, fmt);
    std::fputs("  [ok]   ", stdout);
    std::vfprintf(stdout, fmt, ap);
    std::fputc('\n', stdout);
    va_end(ap);
}

void warn(const char *fmt, ...) noexcept
{
    va_list ap;
    va_start(ap, fmt);
    std::fputs("  [warn] ", stdout);
    std::vfprintf(stdout, fmt, ap);
    std::fputc('\n', stdout);
    va_end(ap);
    ++g_warn;
}

void fail(const char *fmt, ...) noexcept
{
    va_list ap;
    va_start(ap, fmt);
    std::fputs("  [FAIL] ", stdout);
    std::vfprintf(stdout, fmt, ap);
    std::fputc('\n', stdout);
    va_end(ap);
    ++g_fail;
}

void usage()
{
    std::fputs(
        "jr_hw_verify -- 上机前自检（只读；不使能、不发控制帧）\n"
        "\n"
        "用法：\n"
        "  jr_hw_verify --config <file> [选项]\n"
        "\n"
        "选项：\n"
        "  --config <file>        必填：被检配置（DESIGN §6.4 的 YAML）\n"
        "  --probe <N>            节点发现的主动探测上限（默认 16；0 = 仅被动听心跳）\n"
        "  --allow-uncalibrated   未标定只告警（默认：算失败——上机前必须知道）\n"
        "  --allow-shared         允许与其它 master 并行（调试用；单 master 纪律会告警）\n"
        "  --lock-dir <dir>       覆盖配置里的锁目录（多用户/容器里常用）\n"
        "  --quiet                只压 [ok] 行（告警与失败永远打印）\n"
        "  -h, --help             本帮助\n"
        "\n"
        "检查层次（每层独立给结论）：\n"
        "  L1 配置加载与校验        YAML 能否被节点加载；§6.4 校验是否通过\n"
        "  L2 打开总线              锁（谁持有）、SDK ABI 自检、HAL 打开\n"
        "  L3 节点发现              配置里声明的 node_id 是否都在应答；有无配置外节点\n"
        "  L4 设备身份/量程/标定     fw/hw/serial、gear/量程（**来自设备读回**）、标定标志\n"
        "  L5 心跳与看门狗           heartbeat_rate_ms（0 = 不发心跳）、break_timeout（0 = **禁用**）\n"
        "  L6 描述符                下载/缓存、CRC、端点数\n"
        "\n"
        "⚠ 本工具**不验证**闭环控制（不使能、不发控制帧），也不验证实时性能。\n"
        "⚠ 与运行中的 `jr_bus` 节点/jsdk-cli **互斥**（单 master 锁会拦）——\n"
        "   节点在跑时请改用它的服务（`jr_ctl`，走服务）。\n",
        stdout);
}

/** 极简参数解析：只认长选项，**未知选项即报错**（不静默忽略，免得用户以为生效了）。 */
bool parse_args(int argc, char **argv, Options *o, int *exit_code)
{
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        const auto need_value = [&](const char *what) -> const char * {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "jr_hw_verify: %s 需要一个值\n", what);
                *exit_code = kExitUsage;
                return nullptr;
            }
            return argv[++i];
        };
        if (a == "-h" || a == "--help") {
            usage();
            *exit_code = kExitOk;
            return false;
        } else if (a == "--quiet") {
            o->quiet = true;
        } else if (a == "--config") {
            const char *v = need_value("--config");
            if (v == nullptr) return false;
            o->config = v;
        } else if (a == "--probe") {
            const char *v = need_value("--probe");
            if (v == nullptr) return false;
            char *end = nullptr;
            const unsigned long n = std::strtoul(v, &end, 10);
            if (end == nullptr || *end != '\0' || n > 255ul) {
                std::fprintf(stderr, "jr_hw_verify: --probe 需要 0..255 的整数（得到 '%s'）\n", v);
                *exit_code = kExitUsage;
                return false;
            }
            o->probe = static_cast<unsigned>(n);
        } else if (a == "--lock-dir") {
            const char *v = need_value("--lock-dir");
            if (v == nullptr) return false;
            o->lock_dir = v;
        } else if (a == "--allow-uncalibrated") {
            o->allow_uncalibrated = true;
        } else if (a == "--allow-shared") {
            o->allow_shared = true;
        } else {
            std::fprintf(stderr, "jr_hw_verify: 未知选项 '%s'（--help 看用法）\n", a.c_str());
            *exit_code = kExitUsage;
            return false;
        }
    }
    if (o->config.empty()) {
        std::fputs("jr_hw_verify: 缺少 --config（--help 看用法）\n", stderr);
        *exit_code = kExitUsage;
        return false;
    }
    return true;
}

}  // namespace

int main(int argc, char **argv)
{
    Options opt;
    int exit_code = kExitOk;
    if (!parse_args(argc, argv, &opt, &exit_code)) return exit_code;
    g_quiet = opt.quiet;

    std::printf("jr_hw_verify: 配置 '%s'（probe=%u）\n", opt.config.c_str(), opt.probe);

    /* ---- L1 配置加载与校验 ---- */
    std::puts("[L1] 配置加载与校验");
    jr::Config cfg;
    jr::Result res;
    jr::YamlLoadReport rep;
    jr::Status st = jr::load_config_yaml(opt.config.c_str(), &cfg, &res, &rep);
    if (st != jr::Status::kOk) {
        fail("load_config_yaml 失败：%s (%s)", res.message, jr::to_string(st));
        std::printf("\n汇总：%u 通过 / %u 告警 / %u 失败\n", g_pass, g_warn, g_fail);
        return kExitFail;
    }
    pass("YAML 可加载（%u 总线 / %u 关节）", cfg.bus_count, jr::total_joint_count(cfg));
    if (rep.notes.text[0] != '\0') {
        std::printf("  [note] 加载器备注：%s\n", rep.notes.text);
        /* 备注里有真问题（例如 node_id > 7 无法广播）——但它们是"能跑，只是有代价"，
           所以按 warn 计数，不让它把 L1 判死。 */
        ++g_warn;
    }
    if (rep.node_keys > 0u) {
        warn("配置里有 %u 个节点层键（本工具不用，但节点/组件会用）：%s", rep.node_keys,
             rep.node_keys_text);
    }

    if (cfg.bus_count == 0u) {
        fail("配置里没有任何总线");
    }

    /* ---- 逐总线：L2~L6 ---- */
    for (unsigned b = 0u; b < cfg.bus_count; ++b) {
        const jr::BusCfg &bc = cfg.buses[b];
        std::printf("[总线 %u/%u] '%s'（%s '%s'，%u 关节）\n", b + 1u, cfg.bus_count, bc.name,
                    jr::to_string(bc.hal), bc.channel, bc.joint_count);

        std::puts("[L2] 打开总线（锁 / ABI / HAL）");
        jr::rt::BusRuntime bus;
        jr::rt::OpenOptions oo;
        oo.enable_lock = cfg.lock.enabled;
        oo.allow_shared_lock = opt.allow_shared || cfg.lock.allow_shared;
        oo.lock_dir = opt.lock_dir.empty() ? nullptr : opt.lock_dir.c_str();
        if (oo.lock_dir == nullptr && cfg.lock.lock_dir[0] != '\0') {
            oo.lock_dir = cfg.lock.lock_dir;
        }
        st = bus.open(bc, 1000000u, oo, &res);
        if (st != jr::Status::kOk) {
            /* 锁被占是**环境**问题而不是设备问题：单独给 2（脚本据此区分"换台机器重试"）。 */
            fail("打开总线失败：%s (%s)", res.message, jr::to_string(st));
            std::printf("\n汇总：%u 通过 / %u 告警 / %u 失败\n", g_pass, g_warn, g_fail);
            return (st == jr::Status::kLocked) ? kExitUsage : kExitFail;
        }
        pass("总线已打开（锁：%s）",
             !oo.enable_lock ? "按配置关闭" : (oo.allow_shared_lock ? "**共享**（并行）" : "独占"));

        /* ---- L3 节点发现（读设备身份；量程/标定留给 configure 之后的 L4） ---- */
        std::puts("[L3] 节点发现");
        jr::rt::IdentifyOptions io;
        io.probe_max = static_cast<std::uint8_t>(opt.probe);
        io.read_config = false;
        jr::rt::IdentifyReport ident;
        const jr::Status ist = jr::rt::identify_bus(bus, io, &ident, &res);
        if (ist != jr::Status::kOk) {
            fail("节点发现失败：%s (%s)", res.message, jr::to_string(ist));
            bus.close(jr::ExitAction::kNone, nullptr);
            continue;
        }
        if (!opt.quiet) std::printf("%s", ident.text);
        pass("发现 %u 个节点（probe_max=%u）", ident.count, opt.probe);
        /* 配置里声明 vs 总线上真的在答：**两个方向都要报**（漏报会让人带着错线号上机）。 */
        for (unsigned j = 0u; j < bc.joint_count; ++j) {
            bool found = false;
            for (unsigned k = 0u; k < ident.count; ++k) {
                if (ident.node[k].node_id == bc.joints[j].node_id) { found = true; break; }
            }
            if (!found) {
                fail("关节 '%s' 声明 node_id=%u，但总线上**没有应答**（接线/上电/拨号？）",
                     bc.joints[j].name, static_cast<unsigned>(bc.joints[j].node_id));
            }
        }
        for (unsigned k = 0u; k < ident.count; ++k) {
            if (!ident.node[k].in_config) {
                warn("node_id=%u 在总线上有应答，但**配置里没有它**（多出来的设备）",
                     static_cast<unsigned>(ident.node[k].node_id));
            }
        }

        /* ---- L4 描述符 + 量程 + 标定（configure 会下描述符并读回设备真值） ---- */
        std::puts("[L4] 描述符 / 量程 / 标定（configure）");
        jr::rt::BusReport br;
        const jr::Status cst = bus.configure(&br, &res);
        if (cst != jr::Status::kOk) {
            fail("configure 失败：%s (%s)", res.message, jr::to_string(cst));
            bus.close(jr::ExitAction::kNone, nullptr);
            continue;
        }
        pass("描述符 %u 字节 / %u 端点 / CRC 0x%04x%s", br.desc.total_len, br.desc.endpoint_count,
             static_cast<unsigned>(br.desc.crc), br.desc.from_cache ? "（来自缓存）" : "（本次下载）");
        if (br.desc.total_len == 0u || br.desc.endpoint_count == 0u) {
            fail("描述符为空或没有端点 —— 后续任何按路径读写都会失败");
        }

        unsigned calibrated = 0u;
        for (unsigned j = 0u; j < bc.joint_count; ++j) {
            const jr::rt::JointInfoPOD &ji = br.joint[j];
            const char *jn = bc.joints[j].name;
            if (!br.device[j].valid) {
                fail("关节 '%s'：设备信息读不到（QUERY_DEVICE_INFO 无应答）", jn);
            } else {
                pass("关节 '%s'：fw=0x%08x hw=0x%08x serial=0x%llx classic=%s%s", jn,
                     br.device[j].fw_version, br.device[j].hw_version,
                     static_cast<unsigned long long>(br.device[j].serial),
                     br.device[j].classic ? "yes" : "no",
                     (br.device[j].serial == 0u) ? "（serial=0：Classic 下 0x46 只回 hw+fw，协议如此；要序列号请读 `serial_number` 端点）" : "");
            }
            if (ji.gear_ratio <= 0.0 || ji.mit_max_pos <= 0.0 || ji.mit_max_vel <= 0.0 ||
                ji.mit_max_torque <= 0.0) {
                fail("关节 '%s'：量程读回不合理（gear=%.4f pos=%.4f vel=%.4f trq=%.4f）—— "
                     "**不能拿 0 当量程用**", jn, ji.gear_ratio, ji.mit_max_pos, ji.mit_max_vel,
                     ji.mit_max_torque);
            } else {
                pass("关节 '%s'：gear=%.4f pos=%.4f vel=%.4f trq=%.4f（设备读回）", jn,
                     ji.gear_ratio, ji.mit_max_pos, ji.mit_max_vel, ji.mit_max_torque);
            }
            if (ji.calibrated) {
                ++calibrated;
            } else if (opt.allow_uncalibrated) {
                warn("关节 '%s'：**未标定**（--allow-uncalibrated：只告警）", jn);
            } else {
                fail("关节 '%s'：**未标定**（物理量 API 不可用；先标定，或加 --allow-uncalibrated "
                     "明确表示你接受）", jn);
            }

            /* ---- L5 心跳与看门狗（0 的语义是**禁用**，不是"默认 100 ms"） ---- */
            if (ji.heartbeat_rate_ms == 0u) {
                warn("关节 '%s'：heartbeat_rate_ms=0 ⇒ 设备**不发心跳**（被动发现看不到它，"
                     "只靠发送/响应维持链路判断）", jn);
            }
            if (ji.break_timeout_ms == 0u) {
                if (bc.arm_device_watchdog) {
                    fail("关节 '%s'：配置要求 arm_device_watchdog=true，但设备侧 break_timeout "
                         "读回 0（= **超时保护已禁用**）—— 设备侧兜底并不存在", jn);
                } else {
                    warn("关节 '%s'：设备侧 break_timeout=0 ⇒ **超时保护未武装**"
                         "（靠 controller 侧超时兜底；要设备侧兜底请设 arm_device_watchdog=true）",
                         jn);
                }
            } else {
                pass("关节 '%s'：设备侧看门狗已武装（break_timeout=%u ms）", jn,
                     static_cast<unsigned>(ji.break_timeout_ms));
            }
        }
        if (bc.joint_count > 0u && calibrated == bc.joint_count) {
            pass("全部 %u 个关节已标定", calibrated);
        }

        /* ---- L6 节点名/顺序一致性：配置里的顺序 = 以后快照/数组的下标顺序 ---- */
        std::puts("[L6] 配置一致性");
        if (bc.joint_count > jr::kMaxJointsPerBus) {
            fail("关节数 %u > 每总线上限 %u", bc.joint_count, jr::kMaxJointsPerBus);
        } else {
            pass("关节数 %u ≤ %u", bc.joint_count, jr::kMaxJointsPerBus);
        }
        for (unsigned j = 0u; j < bc.joint_count; ++j) {
            for (unsigned k = j + 1u; k < bc.joint_count; ++k) {
                if (bc.joints[j].node_id == bc.joints[k].node_id) {
                    fail("两个关节都声明了 node_id=%u（'%s' 与 '%s'）—— 同号设备行为不确定",
                         static_cast<unsigned>(bc.joints[j].node_id), bc.joints[j].name,
                         bc.joints[k].name);
                }
            }
        }
        if (br.plan.feasible) {
            pass("总线预算：%s", br.plan.text);
        } else {
            fail("总线预算不可行：%s", br.plan.text);
        }

        bus.close(jr::ExitAction::kNone, nullptr);
    }

    std::printf("\n汇总：%u 通过 / %u 告警 / %u 失败\n", g_pass, g_warn, g_fail);
    if (g_fail == 0u) {
        std::puts("结论：静态/配置层没问题（⚠ 本工具不验证闭环控制与实时性能）");
        return kExitOk;
    }
    std::puts("结论：有失败项，上面每条都点名了关节/节点与原始数据");
    return kExitFail;
}
