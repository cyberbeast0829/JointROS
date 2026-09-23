/**
 * @file    jr_ctl_main.cpp
 * @brief   `jr_ctl` —— **节点运行时**的运维入口：走 ROS 服务，**绝不碰总线**
 *
 * @par 它为什么必须存在（DESIGN §12 首屏那条纪律）
 *  "节点跑着的时候不要再跑 `jsdk-cli`" —— 因为那会变成**两个 master**（单 master 锁会拦，
 *  但客户第一反应往往是去查别的东西）。所以文档必须能指一条明路：**用 `jr_ctl`**。
 *  这个工具就是那条明路：它只调 `jr_bus` 节点的 19 个服务，**一个 CAN 帧都不发**，
 *  因此与运行中的节点天然共存。
 *
 * @par 闸门的口径与节点侧保持一致（不另立一套）
 *  - 写参数 / 点动 / 软复位 / 存 Flash / 改 node_id 都必须 `--confirm`（§8.5）；
 *  - 服务端还有自己的开关（`params.allow_write` / `allow_flash_persist`），
 *    被拒时**原样转述服务端给的原因**（工具不揣测、不美化）。
 *
 * @par 退出码（脚本可判别）
 *  0 = 操作成功；1 = 服务调用成功但 `success=false`；2 = 用法错误；
 *  4 = **服务不可达**（节点没起 / 节点名写错 / 超时）—— 与"操作失败"分开，便于脚本重试。
 */

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "jr_interfaces/srv/calibrate.hpp"
#include "jr_interfaces/srv/export_descriptor.hpp"
#include "jr_interfaces/srv/fault_reset.hpp"
#include "jr_interfaces/srv/get_bus_stats.hpp"
#include "jr_interfaces/srv/get_descriptor_info.hpp"
#include "jr_interfaces/srv/get_device_info.hpp"
#include "jr_interfaces/srv/home.hpp"
#include "jr_interfaces/srv/import_descriptor.hpp"
#include "jr_interfaces/srv/jog.hpp"
#include "jr_interfaces/srv/list_endpoints.hpp"
#include "jr_interfaces/srv/lookup_endpoint.hpp"
#include "jr_interfaces/srv/publish_heartbeat_hint.hpp"
#include "jr_interfaces/srv/read_params.hpp"
#include "jr_interfaces/srv/reset_device.hpp"
#include "jr_interfaces/srv/save_config.hpp"
#include "jr_interfaces/srv/set_enabled.hpp"
#include "jr_interfaces/srv/set_node_id.hpp"
#include "jr_interfaces/srv/set_zero.hpp"
#include "jr_interfaces/srv/write_params.hpp"

namespace {

constexpr int kOk = 0;
constexpr int kOpFailed = 1;
constexpr int kUsage = 2;
constexpr int kUnreachable = 4;

/** 全局开关（子命令各自再解析自己的参数）。 */
struct Opts {
    std::string node;          /* 节点名（= 总线名），必填 */
    int         timeout_ms = 5000;
    bool        confirm = false;
    bool        verbose = false;
};

const char *status_name(std::uint8_t s)
{
    switch (s) {
    case 0: return "OK";
    case 1: return "INVALID_ARGUMENT";
    case 2: return "INVALID_STATE";
    case 3: return "TIMEOUT";
    case 4: return "TRANSPORT";
    case 5: return "PROTOCOL";
    case 6: return "NOT_FOUND";
    case 7: return "NOT_SUPPORTED";
    case 8: return "NO_MEMORY";
    case 9: return "NOT_CALIBRATED";
    case 10: return "LOCKED";
    case 11: return "IO_ERROR";
    case 12: return "UNVERIFIED";
    case 13: return "INTERNAL";
    default: return "?";
    }
}

const char *type_name(std::uint8_t t)
{
    switch (t) {
    case 1: return "bool";
    case 2: return "u8";
    case 3: return "i8";
    case 4: return "u16";
    case 5: return "i16";
    case 6: return "u32";
    case 7: return "i32";
    case 8: return "u64";
    case 9: return "i64";
    case 10: return "f32";
    case 11: return "f64";
    default: return "unsupported";
    }
}

/**
 * 服务类型名 → **服务名**（`GetBusStats` → `get_bus_stats`）。
 *
 * ⚠ 为什么需要这一步：DESIGN §6.2 的表列的是**服务类型**名（PascalCase，就是 .srv 文件名），
 *   而节点注册时用的是 ROS 惯例的 **snake_case** 服务名（`jr_bus_services.cpp` 里的
 *   `"~/get_bus_stats"` 等字面量）。按类型名去调会得到一句"服务不可达"，
 *   而真相是**名字不对** —— 实测就这么错过一轮。
 *
 * ⚠ 这是**第二份名字映射**（第一份在节点的注册代码里）。防漂移靠的是端到端用例：
 *   `test_jr_ctl.sh` 会把 19 个服务**逐个真调一遍**，哪个名字漂了当场红。
 */
std::string snake_case_of(const std::string &camel)
{
    std::string out;
    out.reserve(camel.size() + 4u);
    for (std::size_t i = 0u; i < camel.size(); ++i) {
        const char c = camel[i];
        if (c >= 'A' && c <= 'Z') {
            /* 连续大写（如 ID）当一个词：只在"大写后面跟小写"或"小写后面跟大写"处插下划线。 */
            const bool prev_lower = (i > 0u) && (camel[i - 1u] >= 'a' && camel[i - 1u] <= 'z');
            const bool next_lower =
                (i + 1u < camel.size()) && (camel[i + 1u] >= 'a' && camel[i + 1u] <= 'z');
            if (i > 0u && (prev_lower || next_lower)) out.push_back('_');
            out.push_back(static_cast<char>(c - 'A' + 'a'));
        } else {
            out.push_back(c);
        }
    }
    return out;
}

void usage()
{
    std::fputs(
        "jr_ctl -- 节点运行时的运维入口（走服务；**不发任何 CAN 帧**）\n"
        "\n"
        "用法：\n"
        "  jr_ctl --node <bus> [--timeout-ms N] <子命令> [参数...]\n"
        "\n"
        "读取类：\n"
        "  status                              总线状态 / 统计（BusStatus）\n"
        "  info [--joint NAME]                 设备身份（fw/hw/serial/classic）\n"
        "  desc-info                           描述符信息（长度/CRC/端点数/是否来自缓存）\n"
        "  desc-export <file>                  导出描述符原始 JSON 到文件\n"
        "  ep-list [--filter S] [--max N]      枚举端点（filter: 末尾 * = 前缀，. = 段前缀）\n"
        "  ep-lookup <path>                    查单个端点（id/type/读写权限）\n"
        "  read --paths a,b [--joints x,y]     批量读参数\n"
        "  hb-hint [--rate-ms N] [--persist]   心跳建议值（**只给建议，不改设备**）\n"
        "\n"
        "操作类（会动设备/关节，按 §8.5 要 --confirm）：\n"
        "  enable  [--joints a,b] [--resume]   使能\n"
        "  disable [--joints a,b]              失能（走 SDK 安全序列）\n"
        "  calib   [--joints a,b] [--timeout-ms N]  标定（读回确认）\n"
        "  home    [--joints a,b]              回零\n"
        "  zero    [--joints a,b]              置零（不落 Flash）\n"
        "  save    [--joints a,b] --confirm    存 Flash\n"
        "  reset   [--joints a,b] --confirm    软复位（之后需重新 configure）\n"
        "  node-id <joint> <new_id> [--persist] --confirm   改 node_id\n"
        "  fault-reset [--joints a,b] [--allow-device-reset] --confirm  清故障\n"
        "  jog --joint NAME --pos R [--kp K] [--kd D] [--tau T] --duration-s S --confirm\n"
        "  desc-import <file> [--persist] --confirm\n"
        "  write --path P --value V [--joint x] [--persist] --confirm\n"
        "\n"
        "通用选项：--node <bus>（必填）、--timeout-ms N（默认 5000）、--verbose\n"
        "退出码：0 成功 / 1 操作失败 / 2 用法 / **4 服务不可达**（节点没起或名字错）\n"
        "\n"
        "⚠ 与 `jsdk-cli` 的区别：本工具**只走服务**，因此可以在节点运行时用；\n"
        "   要直连总线做体检/生成配置请用 `jr_hw_verify` / `jr_gen_config`（那时**别**开节点）。\n",
        stdout);
}

/** 参数解析小工具：`--key value` 取值，未知选项即报错（不静默忽略）。 */
struct ArgReader {
    int argc = 0;
    char **argv = nullptr;
    int i = 0;
    bool ok = true;
    std::string err;

    bool next(std::string *out)
    {
        if (i >= argc) {
            ok = false;
            err = "缺少参数";
            return false;
        }
        *out = argv[i++];
        return true;
    }
    bool value(const char *what, std::string *out)
    {
        if (i >= argc) {
            ok = false;
            err = std::string(what) + " 需要一个值";
            return false;
        }
        *out = argv[i++];
        return true;
    }
};

std::vector<std::string> split_list(const std::string &s)
{
    std::vector<std::string> out;
    std::string cur;
    for (std::size_t i = 0u; i <= s.size(); ++i) {
        if (i == s.size() || s[i] == ',') {
            if (!cur.empty()) out.push_back(cur);
            cur.clear();
        } else {
            cur.push_back(s[i]);
        }
    }
    return out;
}

/**
 * 调一个服务并等结果。
 *
 * ⚠ 为什么要模板 + 显式超时：`wait_for_service` 与 `spin_until_future_complete` 都要**有限**等待，
 *   否则节点没起时脚本会**挂住**（现场最讨厌的失败模式：没有任何输出）。
 * @return nullptr = 服务不可达（调用方给退出码 4）
 */
template <typename SvcT>
std::shared_ptr<typename SvcT::Response>
call(rclcpp::Node::SharedPtr node, const std::string &node_name, const std::string &svc,
     const typename SvcT::Request &req, int timeout_ms)
{
    /* ⚠ 路径用**snake_case 服务名**（节点注册时的名字），不是类型名 —— 见 `snake_case_of()`。 */
    const std::string full = "/" + node_name + "/" + snake_case_of(svc);
    auto cli = node->create_client<SvcT>(full);
    if (!cli->wait_for_service(std::chrono::milliseconds(timeout_ms))) {
        std::fprintf(stderr,
                     "jr_ctl: 服务 '%s' 不可达（%d ms）。节点在跑吗？--node 写的是总线名吗？\n"
                     "        （可用 `ros2 node list` / `ros2 service list | grep %s` 核对）\n",
                     full.c_str(), timeout_ms, node_name.c_str());
        return nullptr;
    }
    auto fut = cli->async_send_request(std::make_shared<typename SvcT::Request>(req));
    if (rclcpp::spin_until_future_complete(node, fut, std::chrono::milliseconds(timeout_ms)) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        std::fprintf(stderr, "jr_ctl: 服务 '%s' 调用超时（%d ms）\n", full.c_str(), timeout_ms);
        return nullptr;
    }
    return fut.get();
}

/** 逐关节结果的小表（`JointResult[]` 在多个服务里都是同一形状）。 */
template <typename ResT>
void print_joint_results(const ResT &r)
{
    for (const auto &it : r.results) {
        std::printf("  %-16s %-16s %s%s\n", it.name.c_str(), status_name(it.status),
                    it.success ? "" : "✗ ", it.message.c_str());
    }
}

/* ------------------------------------------------------------------------- */
/* 子命令：读取类                                                             */
/* ------------------------------------------------------------------------- */

int cmd_status(rclcpp::Node::SharedPtr node, const Opts &o)
{
    jr_interfaces::srv::GetBusStats::Request req;
    const auto res = call<jr_interfaces::srv::GetBusStats>(node, o.node, "GetBusStats", req,
                                                           o.timeout_ms);
    if (res == nullptr) return kUnreachable;
    std::printf("%s\n", res->message.c_str());
    if (!res->success) return kOpFailed;
    const auto &s = res->status;
    std::printf("  bus=%s link_up=%d nodes_online=%u degraded=%d\n", s.bus.c_str(),
                s.link_up ? 1 : 0, static_cast<unsigned>(s.nodes_online), s.degraded ? 1 : 0);
    std::printf("  tx=%u rx=%u tx_failed=%u rx_dropped=%u keepalive=%u link_errors=%u\n",
                s.tx_frames, s.rx_frames, s.tx_failed, s.rx_dropped, s.keepalive_sent,
                s.link_errors);
    std::printf("  last_rx_age=%u ms hal_bus_flags=0x%08x load≈%.1f%% tick_overruns=%lu "
                "cmd_overwrites=%lu\n",
                s.last_rx_age_ms, s.hal_bus_flags, static_cast<double>(s.bus_load_estimate) * 100.0,
                static_cast<unsigned long>(s.tick_overruns),
                static_cast<unsigned long>(s.command_overwrites));
    if (s.note[0] != '\0') std::printf("  note: %s\n", s.note.c_str());
    return kOk;
}

int cmd_info(rclcpp::Node::SharedPtr node, const Opts &o, const std::string &joint)
{
    jr_interfaces::srv::GetDeviceInfo::Request req;
    req.bus = o.node;
    req.joint = joint;
    const auto res = call<jr_interfaces::srv::GetDeviceInfo>(node, o.node, "GetDeviceInfo", req,
                                                            o.timeout_ms);
    if (res == nullptr) return kUnreachable;
    std::printf("%s\n", res->message.c_str());
    if (!res->success) return kOpFailed;
    std::printf("  fw=0x%08x hw=0x%08x serial=0x%llx classic=%d valid=%d\n", res->info.fw_version,
                res->info.hw_version, static_cast<unsigned long long>(res->info.serial),
                res->info.classic ? 1 : 0, res->info.valid ? 1 : 0);
    return kOk;
}

int cmd_desc_info(rclcpp::Node::SharedPtr node, const Opts &o)
{
    jr_interfaces::srv::GetDescriptorInfo::Request req;
    req.bus = o.node;
    const auto res = call<jr_interfaces::srv::GetDescriptorInfo>(node, o.node, "GetDescriptorInfo",
                                                                req, o.timeout_ms);
    if (res == nullptr) return kUnreachable;
    std::printf("%s\n", res->message.c_str());
    if (!res->success) return kOpFailed;
    const auto &d = res->info;
    std::printf("  total_len=%u json_bytes=%u crc=0x%04x fw=0x%08x\n", d.total_len, d.json_bytes,
                static_cast<unsigned>(d.crc), d.fw_version);
    std::printf("  endpoints=%u parsed=%u frames_rx=%u complete=%d shared_hit=%d from_cache=%d\n",
                d.endpoint_count, d.parsed_total, d.frames_rx, d.complete ? 1 : 0,
                d.shared_hit ? 1 : 0, d.from_cache ? 1 : 0);
    if (o.verbose) std::printf("  cache_path=%s\n", d.cache_path.c_str());
    return kOk;
}

int cmd_desc_export(rclcpp::Node::SharedPtr node, const Opts &o, const std::string &path)
{
    jr_interfaces::srv::ExportDescriptor::Request req;
    req.bus = o.node;
    const auto res = call<jr_interfaces::srv::ExportDescriptor>(node, o.node, "ExportDescriptor", req,
                                                               o.timeout_ms);
    if (res == nullptr) return kUnreachable;
    std::printf("%s\n", res->message.c_str());
    if (!res->success) return kOpFailed;
    std::ofstream f(path, std::ios::binary);
    if (!f) {
        std::fprintf(stderr, "jr_ctl: 写 '%s' 失败\n", path.c_str());
        return kOpFailed;
    }
    f.write(reinterpret_cast<const char *>(res->data.data()),
            static_cast<std::streamsize>(res->data.size()));
    std::printf("  [ok] 已写出 %zu 字节到 %s（crc=0x%04x fw=0x%08x）\n", res->data.size(), path.c_str(),
                static_cast<unsigned>(res->crc), res->fw_version);
    return kOk;
}

int cmd_desc_import(rclcpp::Node::SharedPtr node, const Opts &o, const std::string &path,
                    bool persist)
{
    if (!o.confirm) {
        std::fputs("jr_ctl: desc-import 会改设备描述符，必须 --confirm（见 §8.5）\n", stderr);
        return kUsage;
    }
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        std::fprintf(stderr, "jr_ctl: 读 '%s' 失败\n", path.c_str());
        return kUsage;
    }
    std::vector<std::uint8_t> data((std::istreambuf_iterator<char>(f)),
                                   std::istreambuf_iterator<char>());
    jr_interfaces::srv::ImportDescriptor::Request req;
    req.bus = o.node;
    req.data = data;
    req.persist = persist;
    req.confirm = true;
    const auto res = call<jr_interfaces::srv::ImportDescriptor>(node, o.node, "ImportDescriptor", req,
                                                              o.timeout_ms);
    if (res == nullptr) return kUnreachable;
    std::printf("%s\n", res->message.c_str());
    std::printf("  crc=0x%04x device_reconfigured=%d\n", static_cast<unsigned>(res->crc),
                res->device_reconfigured ? 1 : 0);
    return res->success ? kOk : kOpFailed;
}

int cmd_ep_list(rclcpp::Node::SharedPtr node, const Opts &o, const std::string &filter,
                unsigned max_n)
{
    jr_interfaces::srv::ListEndpoints::Request req;
    req.bus = o.node;
    req.filter = filter;
    req.max = max_n;
    const auto res = call<jr_interfaces::srv::ListEndpoints>(node, o.node, "ListEndpoints", req,
                                                            o.timeout_ms);
    if (res == nullptr) return kUnreachable;
    std::printf("%s\n", res->message.c_str());
    if (!res->success) return kOpFailed;
    for (const auto &e : res->endpoints) {
        std::printf("  %-52s id=%-5u %-11s %s%s\n", e.path.c_str(), static_cast<unsigned>(e.id),
                    type_name(e.type), e.readable ? "r" : "-", e.writable ? "w" : "-");
    }
    if (res->truncated) {
        /* 显式告知截断：静默少给几条会让人以为"设备就这些端点"。 */
        std::printf("  ⚠ 结果被 --max 截断（共 %zu 条）。加 --max 或细化 --filter。\n",
                    res->endpoints.size());
    }
    return kOk;
}

int cmd_ep_lookup(rclcpp::Node::SharedPtr node, const Opts &o, const std::string &path)
{
    jr_interfaces::srv::LookupEndpoint::Request req;
    req.bus = o.node;
    req.path = path;
    const auto res = call<jr_interfaces::srv::LookupEndpoint>(node, o.node, "LookupEndpoint", req,
                                                             o.timeout_ms);
    if (res == nullptr) return kUnreachable;
    std::printf("%s\n", res->message.c_str());
    /* ⚠ `found=false` 要在 `success=false` 之前打：服务对"未命中"会返回 success=false
       （消息里也写了原因），先 return 就看不到这个**脚本可判别**的字段了 —— 实测踩过。 */
    if (!res->found) {
        std::printf("  found=false（路径不存在；工具**不做模糊匹配**，请用 ep-list 找）\n");
    }
    if (!res->success) return kOpFailed;
    std::printf("  %s id=%u type=%s access=%s%s\n", res->endpoint.path.c_str(),
                static_cast<unsigned>(res->endpoint.id), type_name(res->endpoint.type),
                res->endpoint.readable ? "r" : "-", res->endpoint.writable ? "w" : "-");
    return kOk;
}

int cmd_read(rclcpp::Node::SharedPtr node, const Opts &o, const std::vector<std::string> &joints,
             const std::vector<std::string> &paths)
{
    jr_interfaces::srv::ReadParams::Request req;
    req.joints = joints;
    req.paths = paths;
    const auto res = call<jr_interfaces::srv::ReadParams>(node, o.node, "ReadParams", req,
                                                        o.timeout_ms);
    if (res == nullptr) return kUnreachable;
    std::printf("%s\n", res->message.c_str());
    if (!res->success) return kOpFailed;
    for (const auto &v : res->items) {
        std::printf("  %-12s %-44s %-11s ", v.joint.c_str(), v.path.c_str(), type_name(v.type));
        switch (v.type) {
        case 1: std::printf("%s\n", v.bool_value ? "true" : "false"); break;
        case 8: case 6: std::printf("%llu\n", static_cast<unsigned long long>(v.uint64_value)); break;
        case 2: case 3: case 4: case 5: case 7: case 9:
            std::printf("%lld\n", static_cast<long long>(v.int64_value)); break;
        case 10: case 11: std::printf("%.6g\n", v.double_value); break;
        default:
            /* 不可读的类型：如实说"读不了"，不编一个值出来。 */
            std::printf("(%s)\n", status_name(v.status));
            break;
        }
        if (!v.message.empty()) std::printf("      ↳ %s\n", v.message.c_str());
    }
    return kOk;
}

int cmd_write(rclcpp::Node::SharedPtr node, const Opts &o, const std::string &joint,
              const std::string &path, const std::string &value, bool persist)
{
    if (!o.confirm) {
        std::fputs("jr_ctl: write 会改设备参数，必须 --confirm（见 §8.5）。"
                   "（写闸门在服务端也有一道：params.allow_write）\n",
                   stderr);
        return kUsage;
    }
    jr_interfaces::srv::WriteParams::Request req;
    jr_interfaces::msg::ParamWrite w;
    w.joint = joint;
    w.path = path;
    /* 类型留 0：让节点**查描述符**决定类型与宽度（不猜）。
       传文本而不是数字：节点侧的编码器会按端点声明的类型装箱，超值域直接拒。 */
    w.type = 0;
    w.text_value = value;
    req.writes.push_back(w);
    req.persist = persist;
    req.confirm = true;
    const auto res = call<jr_interfaces::srv::WriteParams>(node, o.node, "WriteParams", req,
                                                         o.timeout_ms);
    if (res == nullptr) return kUnreachable;
    std::printf("%s\n", res->message.c_str());
    for (const auto &r : res->results) {
        std::printf("  %-12s %-44s type=%-11s status=%-16s verified=%d consumed=%d persisted=%d\n",
                    r.joint.c_str(), r.path.c_str(), type_name(r.type), status_name(r.status),
                    r.verified ? 1 : 0, r.consumed_by_firmware ? 1 : 0, r.persisted ? 1 : 0);
        std::printf("      requested=%s value=%s\n",
                    (r.type == 10 || r.type == 11)
                        ? std::to_string(r.requested_double).c_str()
                        : std::to_string(r.requested_int64).c_str(),
                    (r.type == 10 || r.type == 11) ? std::to_string(r.value_double).c_str()
                                                   : std::to_string(r.value_int64).c_str());
        if (!r.message.empty()) std::printf("      ↳ %s\n", r.message.c_str());
    }
    std::printf("  all_verified=%d\n", res->all_verified ? 1 : 0);
    return res->success ? kOk : kOpFailed;
}

int cmd_hb_hint(rclcpp::Node::SharedPtr node, const Opts &o, unsigned rate_ms, bool persist)
{
    jr_interfaces::srv::PublishHeartbeatHint::Request req;
    req.bus = o.node;
    req.rate_ms = rate_ms;
    req.persist = persist;
    const auto res = call<jr_interfaces::srv::PublishHeartbeatHint>(node, o.node,
                                                                  "PublishHeartbeatHint", req,
                                                                  o.timeout_ms);
    if (res == nullptr) return kUnreachable;
    std::printf("%s\n", res->message.c_str());
    if (!res->success) return kOpFailed;
    std::printf("  current=%u ms suggested=%u ms extra_frames/s=%u load_delta≈%.2f%%\n",
                res->current_rate_ms, res->suggested_rate_ms, res->extra_frames_per_s,
                static_cast<double>(res->bus_load_delta) * 100.0);
    std::printf("  device_changed=%d（本服务**不修改设备**）\n", res->device_changed ? 1 : 0);
    if (!res->impact.empty()) std::printf("  impact: %s\n", res->impact.c_str());
    return kOk;
}

/* ------------------------------------------------------------------------- */
/* 子命令：操作类（动关节/设备）                                              */
/* ------------------------------------------------------------------------- */

int cmd_set_enabled(rclcpp::Node::SharedPtr node, const Opts &o,
                    const std::vector<std::string> &joints, bool enable, bool resume)
{
    jr_interfaces::srv::SetEnabled::Request req;
    req.joints = joints;
    req.enable = enable;
    req.resume = resume;
    req.timeout_ms = 0u;
    const auto res = call<jr_interfaces::srv::SetEnabled>(node, o.node, "SetEnabled", req,
                                                        o.timeout_ms + 1000);
    if (res == nullptr) return kUnreachable;
    std::printf("%s\n", res->message.c_str());
    print_joint_results(*res);
    return res->success ? kOk : kOpFailed;
}

int cmd_calib_or_home(rclcpp::Node::SharedPtr node, const Opts &o,
                      const std::vector<std::string> &joints, bool calib)
{
    if (calib) {
        jr_interfaces::srv::Calibrate::Request req;
        req.joints = joints;
        req.timeout_ms = 0u;   /* 0 = SDK 默认（120 s）；标定最坏很久 */
        const auto res = call<jr_interfaces::srv::Calibrate>(node, o.node, "Calibrate", req,
                                                           o.timeout_ms + 200000);
        if (res == nullptr) return kUnreachable;
        std::printf("%s\n", res->message.c_str());
        print_joint_results(*res);
        return res->success ? kOk : kOpFailed;
    }
    jr_interfaces::srv::Home::Request req;
    req.joints = joints;
    req.timeout_ms = 0u;
    const auto res = call<jr_interfaces::srv::Home>(node, o.node, "Home", req, o.timeout_ms + 20000);
    if (res == nullptr) return kUnreachable;
    std::printf("%s\n", res->message.c_str());
    print_joint_results(*res);
    return res->success ? kOk : kOpFailed;
}

int cmd_zero(rclcpp::Node::SharedPtr node, const Opts &o, const std::vector<std::string> &joints)
{
    jr_interfaces::srv::SetZero::Request req;
    req.joints = joints;
    const auto res = call<jr_interfaces::srv::SetZero>(node, o.node, "SetZero", req, o.timeout_ms);
    if (res == nullptr) return kUnreachable;
    std::printf("%s\n", res->message.c_str());
    print_joint_results(*res);
    if (!res->persisted) std::printf("  persisted=false（本服务明确不写 Flash）\n");
    return res->success ? kOk : kOpFailed;
}

int cmd_save(rclcpp::Node::SharedPtr node, const Opts &o, const std::vector<std::string> &joints)
{
    if (!o.confirm) {
        std::fputs("jr_ctl: save 会写设备 Flash（寿命有限），必须 --confirm（见 §8.5）\n", stderr);
        return kUsage;
    }
    jr_interfaces::srv::SaveConfig::Request req;
    req.joints = joints;
    req.confirm = true;
    const auto res = call<jr_interfaces::srv::SaveConfig>(node, o.node, "SaveConfig", req,
                                                        o.timeout_ms + 20000);
    if (res == nullptr) return kUnreachable;
    std::printf("%s\n", res->message.c_str());
    print_joint_results(*res);
    return res->success ? kOk : kOpFailed;
}

int cmd_reset(rclcpp::Node::SharedPtr node, const Opts &o, const std::vector<std::string> &joints)
{
    if (!o.confirm) {
        std::fputs("jr_ctl: reset 会软复位设备（之后需重新 configure），必须 --confirm\n", stderr);
        return kUsage;
    }
    jr_interfaces::srv::ResetDevice::Request req;
    req.joints = joints;
    req.confirm = true;
    const auto res = call<jr_interfaces::srv::ResetDevice>(node, o.node, "ResetDevice", req,
                                                         o.timeout_ms + 20000);
    if (res == nullptr) return kUnreachable;
    std::printf("%s\n", res->message.c_str());
    print_joint_results(*res);
    std::printf("  advice=%u\n", static_cast<unsigned>(res->advice));
    return res->success ? kOk : kOpFailed;
}

int cmd_fault_reset(rclcpp::Node::SharedPtr node, const Opts &o,
                    const std::vector<std::string> &joints, bool allow_device_reset)
{
    if (!o.confirm) {
        std::fputs("jr_ctl: fault-reset 会向设备送控制序列（不动电机），按 §8.5 仍需 --confirm\n",
                   stderr);
        return kUsage;
    }
    jr_interfaces::srv::FaultReset::Request req;
    req.joints = joints;
    req.allow_device_reset = allow_device_reset;
    req.confirm = true;
    const auto res = call<jr_interfaces::srv::FaultReset>(node, o.node, "FaultReset", req,
                                                        o.timeout_ms + 30000);
    if (res == nullptr) return kUnreachable;
    std::printf("%s\n", res->message.c_str());
    print_joint_results(*res);
    std::printf("  advice=%u（0=none，非 0 时看服务 message 的说明）\n",
                static_cast<unsigned>(res->advice));
    return res->success ? kOk : kOpFailed;
}

int cmd_node_id(rclcpp::Node::SharedPtr node, const Opts &o, const std::string &joint,
                unsigned new_id, bool persist)
{
    if (!o.confirm) {
        std::fputs("jr_ctl: node-id 改的是通讯配置，必须 --confirm（§8.5）\n", stderr);
        return kUsage;
    }
    if (joint.empty()) {
        std::fputs("jr_ctl: node-id 需要 <joint> <new_id>（用法：--help）\n", stderr);
        return kUsage;
    }
    if (new_id < 1u || new_id > 254u) {
        std::fprintf(stderr, "jr_ctl: new_id 需要 1..254（0 会让设备完全不回复）\n");
        return kUsage;
    }
    jr_interfaces::srv::SetNodeId::Request req;
    req.name = joint;
    req.new_id = static_cast<std::uint8_t>(new_id);
    req.persist = persist;
    req.confirm = true;
    const auto res = call<jr_interfaces::srv::SetNodeId>(node, o.node, "SetNodeId", req,
                                                       o.timeout_ms + 10000);
    if (res == nullptr) return kUnreachable;
    std::printf("%s\n", res->message.c_str());
    std::printf("  advice=%u，persist=%d\n", static_cast<unsigned>(res->advice), persist ? 1 : 0);
    return res->success ? kOk : kOpFailed;
}

int cmd_jog(rclcpp::Node::SharedPtr node, const Opts &o, const std::string &joint, double pos,
            double kp, double kd, double tau, double duration_s)
{
    if (!o.confirm) {
        std::fputs("jr_ctl: jog 会让关节**真的动**，必须 --confirm（与 jsdk-cli 的 --yes 同义）\n",
                   stderr);
        return kUsage;
    }
    if (duration_s > 10.0) {
        std::fprintf(stderr, "jr_ctl: duration-s 上限 10 s（服务端也会拒）\n");
        return kUsage;
    }
    jr_interfaces::srv::Jog::Request req;
    req.joint = joint;
    req.position = pos;
    req.kp = kp;
    req.kd = kd;
    req.torque = tau;
    req.duration_s = duration_s;
    req.confirm = true;
    const auto res = call<jr_interfaces::srv::Jog>(node, o.node, "Jog", req,
                                                o.timeout_ms + static_cast<int>(duration_s * 1000.0) +
                                                    5000);
    if (res == nullptr) return kUnreachable;
    std::printf("%s\n", res->message.c_str());
    std::printf("  actual_duration=%.2f s exit_reason=%s\n",
                static_cast<double>(res->actual_duration_s), res->exit_reason.c_str());
    return res->success ? kOk : kOpFailed;
}

}  // namespace

int main(int argc, char **argv)
{
    if (argc < 2) {
        usage();
        return kUsage;
    }
    /* `--help` 先看：不初始化 rclcpp（否则 --help 也要等 DDS 起来）。 */
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "-h") == 0 || std::strcmp(argv[i], "--help") == 0) {
            usage();
            return kOk;
        }
    }

    Opts o;
    int i = 1;
    std::string sub;
    /* 先扫通用选项（必须在子命令之前）。 */
    for (; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--node" && i + 1 < argc) { o.node = argv[++i]; }
        else if (a == "--timeout-ms" && i + 1 < argc) { o.timeout_ms = std::atoi(argv[++i]); }
        else if (a == "--verbose") { o.verbose = true; }
        else { sub = a; ++i; break; }
    }
    if (o.node.empty() || sub.empty()) {
        std::fputs("jr_ctl: 需要 --node <bus> 与一个子命令（--help 看用法）\n", stderr);
        return kUsage;
    }

    ArgReader ar;
    ar.argc = argc;
    ar.argv = argv;
    ar.i = i;

    /* 子命令自己的参数：先解析出来，再决定调哪个服务。 */
    std::string joint, path, value, file, filter;
    std::vector<std::string> joints;
    std::vector<std::string> paths;
    double pos = 0.0, kp = 0.0, kd = 0.0, tau = 0.0, duration_s = 0.0;
    unsigned max_n = 0u, new_id = 0u, rate_ms = 0u;
    bool persist = false, resume = false, allow_device_reset = false;

    while (ar.i < argc) {
        std::string a;
        if (!ar.next(&a)) break;
        if (a == "--joint" || a == "--name") { if (!ar.value("--joint", &joint)) break; }
        else if (a == "--joints") { std::string v; if (!ar.value("--joints", &v)) break; joints = split_list(v); }
        else if (a == "--paths") { std::string v; if (!ar.value("--paths", &v)) break; paths = split_list(v); }
        else if (a == "--path") { if (!ar.value("--path", &path)) break; }
        else if (a == "--value") { if (!ar.value("--value", &value)) break; }
        else if (a == "--filter") { if (!ar.value("--filter", &filter)) break; }
        else if (a == "--max") { std::string v; if (!ar.value("--max", &v)) break; max_n = static_cast<unsigned>(std::strtoul(v.c_str(), nullptr, 10)); }
        else if (a == "--rate-ms") { std::string v; if (!ar.value("--rate-ms", &v)) break; rate_ms = static_cast<unsigned>(std::strtoul(v.c_str(), nullptr, 10)); }
        else if (a == "--pos") { std::string v; if (!ar.value("--pos", &v)) break; pos = std::strtod(v.c_str(), nullptr); }
        else if (a == "--kp") { std::string v; if (!ar.value("--kp", &v)) break; kp = std::strtod(v.c_str(), nullptr); }
        else if (a == "--kd") { std::string v; if (!ar.value("--kd", &v)) break; kd = std::strtod(v.c_str(), nullptr); }
        else if (a == "--tau") { std::string v; if (!ar.value("--tau", &v)) break; tau = std::strtod(v.c_str(), nullptr); }
        else if (a == "--duration-s") { std::string v; if (!ar.value("--duration-s", &v)) break; duration_s = std::strtod(v.c_str(), nullptr); }
        else if (a == "--confirm") { o.confirm = true; }
        else if (a == "--persist") { persist = true; }
        else if (a == "--resume") { resume = true; }
        else if (a == "--allow-device-reset") { allow_device_reset = true; }
        else if (a == "--timeout-ms") { std::string v; if (!ar.value("--timeout-ms", &v)) break; o.timeout_ms = std::atoi(v.c_str()); }
        else if (!a.empty() && a[0] == '-') {
            std::fprintf(stderr, "jr_ctl: 未知选项 '%s'（--help 看用法）\n", a.c_str());
            return kUsage;
        } else if (file.empty()) { file = a; }
        else if (new_id == 0u) { new_id = static_cast<unsigned>(std::strtoul(a.c_str(), nullptr, 10)); }
        else {
            std::fprintf(stderr, "jr_ctl: 多余参数 '%s'\n", a.c_str());
            return kUsage;
        }
    }
    if (!ar.ok) {
        std::fprintf(stderr, "jr_ctl: %s\n", ar.err.c_str());
        return kUsage;
    }

    /* ⚠ 位置参数要**按子命令**解释：`desc-export/ep-lookup/desc-import` 的第一个位置参数是
       **文件/路径**，而 `node-id` 的第一个位置参数是**关节名**（第二才是 new_id）。
       不区分时 `node-id j1 2` 会把 "j1" 当成 file、joint 留空 → 服务回
       "joint '' is not on bus 'vbusrp'"（报错点离病因很远，实测踩过）。 */
    if (sub == "node-id" && joint.empty() && !file.empty()) {
        joint = file;
        file.clear();
    }

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("jr_ctl");

    int rc = kUsage;
    if (sub == "status") rc = cmd_status(node, o);
    else if (sub == "info") rc = cmd_info(node, o, joint);
    else if (sub == "desc-info") rc = cmd_desc_info(node, o);
    else if (sub == "desc-export") rc = file.empty() ? kUsage : cmd_desc_export(node, o, file);
    else if (sub == "desc-import") rc = file.empty() ? kUsage : cmd_desc_import(node, o, file, persist);
    else if (sub == "ep-list") rc = cmd_ep_list(node, o, filter, max_n);
    else if (sub == "ep-lookup") rc = file.empty() ? kUsage : cmd_ep_lookup(node, o, file);
    else if (sub == "read") rc = paths.empty() ? kUsage : cmd_read(node, o, joints, paths);
    else if (sub == "write") rc = (path.empty() || value.empty()) ? kUsage
                                                                 : cmd_write(node, o, joint, path, value, persist);
    else if (sub == "hb-hint") rc = cmd_hb_hint(node, o, rate_ms, persist);
    else if (sub == "enable") rc = cmd_set_enabled(node, o, joints, true, resume);
    else if (sub == "disable") rc = cmd_set_enabled(node, o, joints, false, false);
    else if (sub == "calib") rc = cmd_calib_or_home(node, o, joints, true);
    else if (sub == "home") rc = cmd_calib_or_home(node, o, joints, false);
    else if (sub == "zero") rc = cmd_zero(node, o, joints);
    else if (sub == "save") rc = cmd_save(node, o, joints);
    else if (sub == "reset") rc = cmd_reset(node, o, joints);
    else if (sub == "fault-reset") rc = cmd_fault_reset(node, o, joints, allow_device_reset);
    else if (sub == "node-id") rc = cmd_node_id(node, o, joint, new_id, persist);
    else if (sub == "jog") rc = cmd_jog(node, o, joint, pos, kp, kd, tau, duration_s);
    else {
        std::fprintf(stderr, "jr_ctl: 未知子命令 '%s'（--help 看用法）\n", sub.c_str());
        rc = kUsage;
    }

    rclcpp::shutdown();
    return rc;
}
