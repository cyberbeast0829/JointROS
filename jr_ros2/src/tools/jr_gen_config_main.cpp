/**
 * @file    jr_gen_config_main.cpp
 * @brief   `jr_gen_config` —— 扫总线 → 生成**能被节点直接加载**的配置（DESIGN §6.4）
 *
 * @par 为什么值得单独一个工具
 *   客户配一台整机最容易错的三件事：node_id 拨错、gear/量程抄错、忘了关节数上限。
 *   这三件都能**从设备读回来**（`configure()` 读回量程 + `QUERY_DEVICE_INFO` 读身份），
 *   所以"让客户手抄"是没必要的错误来源（JointSDK 的硬规矩：**不手抄设备参数**）。
 *
 * @par 它**不猜**什么（读不回来的项，用默认值 + 注释写明，绝不假装是扫描结果）
 *   - `master_id`：设备侧读不回来 → 写默认 1 并在注释里点名"请核对"；
 *   - `is_fd`：没有设备信息时无从判断 → 由设备 `classic` 位推断，注释标明推断来源；
 *   - `stiffness/damping`：这是**控制器增益**（属机械+负载），设备里没有 → **故意不写**；
 *   - 关节名：设备不上报名字 → 默认 `j<node_id>`，可用 `--names` 按发现顺序给出真名。
 *
 * @par 生成之后**当场自检**（不通过就不落盘）
 *   `parse_config_yaml()`（同一个 §6.4 schema）→ `validate_config()` → `plan_bus()`。
 *   为什么要这样：配置文件是"给未来用的输入"，写一个**连自己都加载不了**的文件
 *   比报错危险得多（客户会拿去跑，然后在别处看到一句莫名其妙的报错）。
 *
 * @par 退出码：0 = 已生成且自检通过；1 = 失败（含自检不通过，**文件不会写**）；2 = 用法/环境。
 */

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>
#include <vector>

#include "jr_ros2/jr_config.hpp"
#include "jr_ros2/jr_config_yaml.hpp"
#include "jr_ros2/jr_gen_config_cfg.hpp"
#include "jr_ros2/rt/jr_bus_runtime.hpp"
#include "jr_ros2/rt/jr_identify.hpp"

namespace {

constexpr int kExitOk = 0;
constexpr int kExitFail = 1;
constexpr int kExitUsage = 2;

struct Options {
    std::string if_kind;
    std::string channel;
    std::string bus_name;
    std::string out;              /* 空 = stdout */
    std::string names;            /* "a,b,c"（按发现顺序） */
    std::string lock_dir;
    std::string mode = "mit";
    unsigned    probe = 16u;
    unsigned    master_id = 1u;
    double      rate_hz = 1000.0;
    int         is_fd = -1;       /* -1 = 未指定（Classic 起步，SDK 按对端帧对齐） */
    unsigned    serial_baud = 115200u;  /* 仅 slcan：**串口**速率，不是 CAN 速率 */
    bool        allow_shared = false;
    bool        force = false;
};

void usage()
{
    std::fputs(
        "jr_gen_config -- 扫总线并生成 DESIGN §6.4 的配置（生成前当场自检）\n"
        "\n"
        "用法：\n"
        "  jr_gen_config --if <socketcan|slcan|pcan|virtual> --channel <通道> [选项]\n"
        "\n"
        "选项：\n"
        "  --if <kind>          总线后端（与配置里的 `type` 同义）\n"
        "  --channel <c>        socketcan/pcan/slcan = 接口名；virtual = sim spec\n"
        "                       （例：\"0:id=1,gear=7.75,pmax=12.5,vmax=65,tmax=50,hb=5\"）\n"
        "  --serial-baud <N>    **仅 slcan**：串口速率（默认 115200；不是 CAN 速率）\n"
        "  --name <bus-name>    生成的配置里这条总线叫什么（默认：channel，或 'virt'）\n"
        "  --probe <N>          主动探测上限（默认 16；0 = 仅被动听心跳）\n"
        "  --master-id <M>      主站号（默认 1；⚠ 设备侧读不回来 → 生成文件里会点名要核对）\n"
        "  --is-fd <0|1>        CAN-FD（默认：由设备 classic 位推断，注释里标明）\n"
        "  --names a,b,c        关节名（**按发现顺序**，数量必须与发现数一致）\n"
        "  --mode <m>           关节模式 mit|csp|csv|cst|current（默认 mit）\n"
        "  --rate-hz <N>        tick 率（默认 1000；用于预算自检与 tick_groups）\n"
        "  --out <file>         写到文件（默认 stdout；已存在则拒绝，除非 --force）\n"
        "  --lock-dir <dir>     扫描时用的锁目录\n"
        "  --allow-shared       允许与其它 master 并行扫描（调试用）\n"
        "  -h, --help           本帮助\n"
        "\n"
        "⚠ 扫描会主动发 QUERY_STATUS（probe>0）——真机上是有代价的动作；\n"
        "   节点/jsdk-cli 在跑时**不要**用它（单 master 锁会拦，或者用 --allow-shared 看告警）。\n",
        stdout);
}

bool parse_args(int argc, char **argv, Options *o, int *exit_code)
{
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        const auto value = [&](const char *what) -> const char * {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "jr_gen_config: %s 需要一个值\n", what);
                *exit_code = kExitUsage;
                return nullptr;
            }
            return argv[++i];
        };
        if (a == "-h" || a == "--help") { usage(); *exit_code = kExitOk; return false; }
        else if (a == "--if") { const char *v = value("--if"); if (!v) return false; o->if_kind = v; }
        else if (a == "--channel") { const char *v = value("--channel"); if (!v) return false; o->channel = v; }
        else if (a == "--name") { const char *v = value("--name"); if (!v) return false; o->bus_name = v; }
        else if (a == "--out") { const char *v = value("--out"); if (!v) return false; o->out = v; }
        else if (a == "--names") { const char *v = value("--names"); if (!v) return false; o->names = v; }
        else if (a == "--lock-dir") { const char *v = value("--lock-dir"); if (!v) return false; o->lock_dir = v; }
        else if (a == "--mode") { const char *v = value("--mode"); if (!v) return false; o->mode = v; }
        else if (a == "--rate-hz") {
            const char *v = value("--rate-hz"); if (!v) return false;
            char *end = nullptr; const double d = std::strtod(v, &end);
            if (end == nullptr || *end != '\0' || !(d >= 1.0 && d <= 20000.0)) {
                std::fprintf(stderr, "jr_gen_config: --rate-hz 需要 1..20000（得到 '%s'）\n", v);
                *exit_code = kExitUsage; return false;
            }
            o->rate_hz = d;
        } else if (a == "--probe") {
            const char *v = value("--probe"); if (!v) return false;
            char *end = nullptr; const unsigned long n = std::strtoul(v, &end, 10);
            if (end == nullptr || *end != '\0' || n > 255ul) {
                std::fprintf(stderr, "jr_gen_config: --probe 需要 0..255（得到 '%s'）\n", v);
                *exit_code = kExitUsage; return false;
            }
            o->probe = static_cast<unsigned>(n);
        } else if (a == "--master-id") {
            const char *v = value("--master-id"); if (!v) return false;
            char *end = nullptr; const unsigned long n = std::strtoul(v, &end, 10);
            if (end == nullptr || *end != '\0' || n < 1ul || n > 254ul) {
                std::fprintf(stderr, "jr_gen_config: --master-id 需要 1..254（得到 '%s'）\n", v);
                *exit_code = kExitUsage; return false;
            }
            o->master_id = static_cast<unsigned>(n);
        } else if (a == "--is-fd") {
            const char *v = value("--is-fd"); if (!v) return false;
            if (std::strcmp(v, "0") == 0) o->is_fd = 0;
            else if (std::strcmp(v, "1") == 0) o->is_fd = 1;
            else {
                std::fprintf(stderr, "jr_gen_config: --is-fd 只接受 0 或 1（得到 '%s'）\n", v);
                *exit_code = kExitUsage; return false;
            }
        } else if (a == "--serial-baud") {
            const char *v = value("--serial-baud"); if (!v) return false;
            char *end = nullptr; const unsigned long n = std::strtoul(v, &end, 10);
            if (end == nullptr || *end != '\0' || n < 1200ul || n > 4000000ul) {
                std::fprintf(stderr, "jr_gen_config: --serial-baud 需要 1200..4000000（得到 '%s'）\n", v);
                *exit_code = kExitUsage; return false;
            }
            o->serial_baud = static_cast<unsigned>(n);
        } else if (a == "--allow-shared") { o->allow_shared = true; }
        else if (a == "--force") { o->force = true; }
        else {
            std::fprintf(stderr, "jr_gen_config: 未知选项 '%s'（--help 看用法）\n", a.c_str());
            *exit_code = kExitUsage; return false;
        }
    }
    if (o->if_kind.empty() || o->channel.empty()) {
        std::fputs("jr_gen_config: 必须给 --if 与 --channel（--help 看用法）\n", stderr);
        *exit_code = kExitUsage;
        return false;
    }
    return true;
}

/** YAML 文本里我们**只接受**这种名字：不合规就报错，而不是替客户加引号/转义兜底
 *  （配置是客户要长期维护的东西，名字怪到需要转义时，早点让他看到比悄悄转义好）。 */
bool name_is_plain(const char *s) noexcept
{
    if (s == nullptr || s[0] == '\0') return false;
    for (const char *p = s; *p != '\0'; ++p) {
        const char c = *p;
        const bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                        (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.';
        if (!ok) return false;
    }
    return true;
}

void addf(std::string *out, const char *fmt, ...) noexcept
{
    char buf[512] = {};
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    out->append(buf);
}

}  // namespace

int main(int argc, char **argv)
{
    Options opt;
    int exit_code = kExitOk;
    if (!parse_args(argc, argv, &opt, &exit_code)) return exit_code;

    jr::HalKind hal{};
    if (!jr::hal_kind_from_name(opt.if_kind.c_str(), &hal)) {
        std::fprintf(stderr, "jr_gen_config: 未知 --if '%s'（可选 socketcan|slcan|pcan|virtual）\n",
                     opt.if_kind.c_str());
        return kExitUsage;
    }
    const char *mode_str = opt.mode.c_str();
    jr::CmdMode mode = jr::CmdMode::kMit;
    if (std::strcmp(mode_str, "csp") == 0) mode = jr::CmdMode::kCsp;
    else if (std::strcmp(mode_str, "csv") == 0) mode = jr::CmdMode::kCsv;
    else if (std::strcmp(mode_str, "cst") == 0) mode = jr::CmdMode::kCst;
    else if (std::strcmp(mode_str, "current") == 0) mode = jr::CmdMode::kCurrent;
    else if (std::strcmp(mode_str, "mit") != 0) {
        std::fprintf(stderr, "jr_gen_config: 未知 --mode '%s'（mit|csp|csv|cst|current）\n", mode_str);
        return kExitUsage;
    }

    const std::string bus_name = opt.bus_name.empty()
                                     ? (hal == jr::HalKind::kVirtual ? std::string("virt") : opt.channel)
                                     : opt.bus_name;
    if (!name_is_plain(bus_name.c_str())) {
        std::fprintf(stderr, "jr_gen_config: 总线名 '%s' 只允许 [A-Za-z0-9_.-]\n", bus_name.c_str());
        return kExitUsage;
    }

    /* ---- ① 用默认配置开一条空总线（锁/ABI/零初始化 context 都走 BusRuntime::open） ---- */
    jr::Config cfg = jr::default_config();
    cfg.bus_count = 1;
    /* ⚠ “命令行 → 开总线用的字段”全部交给那个**纯函数**（离线可测，§13.3-47）：
       slcan 的 `serial_baud` 与 `--is-fd` 必须**真的**落进去 —— 以前这两处都漏了，
       于是“生成的配置自己都加载不了” + “在 Classic 设备上按 FD 扫描”。 */
    jr::tools::ScanCfgOpts so;
    so.channel = opt.channel;
    so.bus_name = bus_name;
    so.master_id = opt.master_id;
    so.is_fd = opt.is_fd;
    so.serial_baud = opt.serial_baud;
    cfg.buses[0] = jr::tools::build_scan_bus_cfg(hal, so);
    jr::BusCfg &bc = cfg.buses[0];

    const std::uint32_t period_ns = static_cast<std::uint32_t>(1e9 / opt.rate_hz);

    jr::rt::BusRuntime bus;
    jr::rt::OpenOptions oo;
    oo.enable_lock = cfg.lock.enabled;
    oo.allow_shared_lock = opt.allow_shared || cfg.lock.allow_shared;
    oo.lock_dir = opt.lock_dir.empty() ? nullptr : opt.lock_dir.c_str();
    /* 扫描时关节数还是未知的（本来就靠这次扫描才知道）→ 允许 0 关节打开。 */
    oo.allow_empty_scan_bus = true;

    jr::Result res;
    jr::Status st = bus.open(bc, period_ns, oo, &res);
    if (st != jr::Status::kOk) {
        std::fprintf(stderr, "jr_gen_config: 打开总线失败：%s (%s)\n", res.message, jr::to_string(st));
        return (st == jr::Status::kLocked) ? kExitUsage : kExitFail;
    }

    /* ---- ② 扫描（发现 + configure 读量程/标定） ---- */
    std::printf("jr_gen_config: 扫描 %s '%s'（probe=%u）…\n", jr::to_string(hal), opt.channel.c_str(),
                opt.probe);
    jr::rt::IdentifyOptions io;
    io.probe_max = static_cast<std::uint8_t>(opt.probe);
    io.read_config = true;
    io.period_ns = period_ns;
    jr::rt::IdentifyReport ident;
    const jr::Status ist = jr::rt::identify_bus(bus, io, &ident, &res);
    std::printf("%s", ident.text);
    if (ident.count == 0u) {
        std::fprintf(stderr, "jr_gen_config: 总线上一台设备都没发现（%s）\n",
                     res.message[0] != '\0' ? res.message : "no response");
        bus.close(jr::ExitAction::kNone, nullptr);
        return kExitFail;
    }
    if (ist != jr::Status::kOk) {
        /* 发现了设备，但 configure() 失败（常见：未标定）。量程读不到 ⇒ 生成的 limits 会缺值，
           **不假装**：告诉用户哪一层缺，并把已经读到的身份照常用来生成。 */
        std::fprintf(stderr,
                     "jr_gen_config: ⚠ 量程/标定未读到（%s）—— 生成的配置会缺少 limits，"
                     "请先解决这个问题再上机\n", res.message);
    }
    if (ident.count > jr::kMaxJointsPerBus) {
        std::fprintf(stderr, "jr_gen_config: 发现 %u 个节点 > 每总线上限 %u（一条总线放不下）\n",
                     ident.count, jr::kMaxJointsPerBus);
        bus.close(jr::ExitAction::kNone, nullptr);
        return kExitFail;
    }

    /* ---- ③ 关节名：`--names` 必须**不多不少**（少一个就报错，不替客户编号） ---- */
    std::vector<std::string> names;
    if (!opt.names.empty()) {
        std::string cur;
        for (std::size_t i = 0u; i <= opt.names.size(); ++i) {
            if (i == opt.names.size() || opt.names[i] == ',') {
                names.push_back(cur);
                cur.clear();
            } else {
                cur.push_back(opt.names[i]);
            }
        }
        if (names.size() != ident.count) {
            std::fprintf(stderr, "jr_gen_config: --names 给了 %zu 个名字，但发现 %u 个节点\n",
                         names.size(), ident.count);
            bus.close(jr::ExitAction::kNone, nullptr);
            return kExitUsage;
        }
    } else {
        /* ⚠ 这里必须按 `opt.names` 是否为空来分支，**不能**用 `names.empty()` 当"没给名字"
           的判据：第一次 push 之后 `names.empty()` 就永假，于是第二个关节会去访问
           `names[1]` —— 实测直接 `std::vector` 越界 assert 崩掉（工具崩比报错难查得多）。 */
        for (unsigned k = 0u; k < ident.count; ++k) {
            char tmp[32] = {};
            std::snprintf(tmp, sizeof tmp, "j%u", static_cast<unsigned>(ident.node[k].node_id));
            names.push_back(tmp);
        }
    }
    for (std::size_t k = 0u; k < names.size(); ++k) {
        if (!name_is_plain(names[k].c_str())) {
            std::fprintf(stderr, "jr_gen_config: 关节名 '%s' 只允许 [A-Za-z0-9_.-]\n",
                         names[k].c_str());
            bus.close(jr::ExitAction::kNone, nullptr);
            return kExitUsage;
        }
    }

    /* ---- ④ is_fd：命令行优先；否则由设备 classic 位推断（并注明来源） ---- */
    bool fd = true;
    bool fd_from_device = false;
    if (opt.is_fd >= 0) {
        fd = (opt.is_fd != 0);
    } else {
        bool any_classic = false;
        bool any_known = false;
        for (unsigned k = 0u; k < ident.count; ++k) {
            if (ident.node[k].device.valid) {
                any_known = true;
                if (ident.node[k].device.classic) any_classic = true;
            }
        }
        if (any_known) {
            fd = !any_classic;
            fd_from_device = true;
        }
    }

    /* ---- ⑤ 拼 YAML（与加载器同 schema；注释里写清哪些不是扫描结果） ---- */
    std::time_t now = std::time(nullptr);
    char ts[64] = {};
    std::strftime(ts, sizeof ts, "%Y-%m-%d %H:%M:%S", std::localtime(&now));

    std::string y;
    addf(&y, "# 由 jr_gen_config 生成（%s）\n", ts);
    addf(&y, "#   设备侧读回：gear / pos / vel / trq / 标定标志 / fw / serial\n");
    addf(&y, "#   ⚠ 以下三项**设备读不回来**，请人工核对：\n");
    addf(&y, "#     - master_id=%u（默认值；错了两台主站会争总线）\n", opt.master_id);
    addf(&y, "#     - is_fd=%d（%s）\n", fd ? 1 : 0,
         opt.is_fd >= 0 ? "命令行给定" : (fd_from_device ? "由设备 classic 位推断" : "无设备信息，取默认"));
    addf(&y, "#     - limits 里的 stiffness/damping **故意没写**：那是控制器增益，设备里没有\n");
    addf(&y, "jr:\n");
    addf(&y, "  rt:\n    enabled: %s\n", cfg.rt.enabled ? "true" : "false");
    const jr::TickGroupCfg &g0 = cfg.groups[0];
    addf(&y, "  tick_groups:\n");
    /* ⚠ `cpu`/`priority` 是**有符号** int，默认 -1（不绑核 / 用 rt 默认）——
       按无符号打出来会变成 4294967295，加载器立刻报
       "tick_groups[0].cpu: expected an integer (bad conversion)"。
       这正是生成器"生成→回读"自检存在的意义：它当场抓到了这个 bug。 */
    addf(&y, "    - {name: %s, rate_hz: %.0f, buses: [%s], cpu: %d, priority: %d}\n",
         (g0.name[0] != '\0') ? g0.name : "g0", opt.rate_hz, bus_name.c_str(), g0.cpu, g0.priority);
    addf(&y, "      #  ↑ cpu/priority = -1 表示不绑核/用 rt 段默认；生产建议绑核并核对 CPU 布局\n");
    addf(&y, "  buses:\n");
    addf(&y, "    - name: %s\n", bus_name.c_str());
    addf(&y, "      type: %s\n", jr::to_string(hal));
    if (hal == jr::HalKind::kVirtual) {
        addf(&y, "      spec: \"%s\"\n", opt.channel.c_str());
    } else {
        addf(&y, "      interface: %s\n", opt.channel.c_str());
    }
    /* ⚠ slcan 的 schema **要求** serial_baud（漏了它，生成的配置连我们自己的加载器
       都过不了 —— 加载器的“生成→回读”自检当场拒绍落盘，§13.3-47 ①）。
       同时写清它是**串口**速率，不是 CAN 速率（两个量极易搞混）。 */
    if (hal == jr::HalKind::kSlcan) {
        addf(&y, "      serial_baud: %u         # 串口速率（不是 CAN 速率）\n", opt.serial_baud);
    }
    addf(&y, "      is_fd: %s\n", fd ? "true" : "false");
    addf(&y, "      master_id: %u\n", opt.master_id);
    addf(&y, "      bitrate: {nominal: %u, data: %u}\n", static_cast<unsigned>(bc.nominal_bitrate),
         static_cast<unsigned>(bc.data_bitrate));
    addf(&y, "      joints: [");
    for (unsigned k = 0u; k < ident.count; ++k) addf(&y, "%s%s", (k == 0u) ? "" : ", ", names[k].c_str());
    addf(&y, "]\n");
    addf(&y, "  joints:\n");
    for (unsigned k = 0u; k < ident.count; ++k) {
        addf(&y, "    - {name: %s, bus: %s, node_id: %u, mode: %s}\n", names[k].c_str(),
             bus_name.c_str(), static_cast<unsigned>(ident.node[k].node_id),
             jr::to_string(mode));
    }
    if (ident.config_read) {
        addf(&y, "  limits:        # 量程来自设备读回（**不要手抄**）\n");
        for (unsigned k = 0u; k < ident.count; ++k) {
            const jr::rt::JointInfoPOD &j = ident.node[k].config;
            addf(&y, "    %s:\n", names[k].c_str());
            addf(&y, "      position: [%.4f, %.4f]\n", -j.mit_max_pos, j.mit_max_pos);
            addf(&y, "      velocity: %.4f\n", j.mit_max_vel);
            addf(&y, "      effort: %.4f\n", j.mit_max_torque);
        }
    } else {
        addf(&y, "  # ⚠ limits 未生成：扫描时没读到量程（configure 失败 / --no-config-read）\n");
    }
    addf(&y, "  command:\n    timeout_ms: %u\n    timeout_action: %s\n",
         static_cast<unsigned>(cfg.command.timeout_ms), jr::to_string(cfg.command.on_timeout));
    addf(&y, "  safety:\n");
    addf(&y, "    auto_enable: %s          # 冷启动不自动使能\n", cfg.safety.auto_enable ? "true" : "false");
    addf(&y, "    require_calibrated: %s\n", cfg.safety.require_calibrated ? "true" : "false");
    addf(&y, "    arm_device_watchdog: %s  # true = 让设备侧兜底（会动设备参数）\n",
         bc.arm_device_watchdog ? "true" : "false");
    addf(&y, "    clamp_target: %s\n", bc.clamp_target ? "true" : "false");
    addf(&y, "    on_exit_action: %s\n", jr::to_string(cfg.safety.on_exit));
    addf(&y, "    fault_action: %s\n", jr::to_string(cfg.safety.on_fault));
    addf(&y, "  bus_lock:\n    enabled: true\n    allow_shared: false\n");
    if (cfg.lock.lock_dir[0] != '\0') addf(&y, "    lock_dir: %s\n", cfg.lock.lock_dir);

    bus.close(jr::ExitAction::kNone, nullptr);

    /* ---- ⑥ 自检：用**同一个**加载器读回来（不通过就不落盘） ---- */
    jr::Config check;
    jr::Result cres;
    jr::YamlLoadReport crep;
    const jr::Status pst = jr::parse_config_yaml(y.c_str(), "generated", &check, &cres, &crep);
    if (pst != jr::Status::kOk) {
        std::fprintf(stderr,
                     "jr_gen_config: ⚠ 内部自检失败——生成的 YAML **加载不了**（%s）。"
                     "这是本工具的 bug，文件**没有**写出。\n", cres.message);
        return kExitFail;
    }
    if (check.bus_count != 1u || check.buses[0].joint_count != ident.count) {
        std::fprintf(stderr, "jr_gen_config: ⚠ 自检不一致（总线 %u / 关节 %u，期望 1 / %u）\n",
                     check.bus_count, check.buses[0].joint_count, ident.count);
        return kExitFail;
    }
    const jr::rt::BusPlanResult plan = jr::rt::plan_bus(check.buses[0], opt.rate_hz);
    if (!plan.feasible) {
        std::fprintf(stderr, "jr_gen_config: 生成的配置**预算不可行**：%s\n", plan.text);
        return kExitFail;
    }

    /* ---- ⑦ 落盘 ---- */
    if (opt.out.empty()) {
        std::fputs(y.c_str(), stdout);
    } else {
        std::FILE *f = std::fopen(opt.out.c_str(), opt.force ? "wb" : "wbx");
        if (f == nullptr) {
            std::fprintf(stderr, "jr_gen_config: 写 '%s' 失败（已存在？加 --force）\n", opt.out.c_str());
            return kExitFail;
        }
        std::fwrite(y.data(), 1u, y.size(), f);
        std::fclose(f);
        std::printf("jr_gen_config: 已写出 %s（%zu 字节，自检通过：load+validate+plan_bus）\n",
                    opt.out.c_str(), y.size());
        std::printf("  ⚠ 生成后请复核：master_id / is_fd / 关节名（见文件头注释）\n");
    }
    return kExitOk;
}
