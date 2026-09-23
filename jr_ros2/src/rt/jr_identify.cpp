/**
 * @file    jr_identify.cpp
 * @brief   总线身份识别（发现 + 设备身份 + 量程/标定）
 */

#include "jr_ros2/rt/jr_identify.hpp"

#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "joint_sdk/joint_sdk.h"

#include "jr_sdk_map.hpp"   /* internal::map_sdk_status / advice_from_sdk_text（唯一映射点） */

namespace jr {
namespace rt {
namespace {

/** 往报告文本里追加一行（固定缓冲；写满即停，不静默截断——`Result::set` 同款策略）。 */
void add_line(char *buf, std::size_t cap, const char *fmt, ...) noexcept
{
    std::size_t len = std::strlen(buf);
    if (len + 1u >= cap) return;
    va_list ap;
    va_start(ap, fmt);
    const int n = std::vsnprintf(buf + len, cap - len, fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    if (static_cast<std::size_t>(n) >= cap - len) {
        buf[cap - 1u] = '\0';   /* 截断就停在边界，别越界 */
        return;
    }
    len += static_cast<std::size_t>(n);
    if (len + 1u < cap) {
        buf[len] = '\n';
        buf[len + 1u] = '\0';
    }
}

/** 配置里是否已有该 node_id；有则返回其下标与关节名（用于交叉核对与复用已有句柄）。 */
bool find_in_config(const BusCfg &bus, std::uint8_t node_id, unsigned *index_out,
                    const char **name_out) noexcept
{
    for (unsigned i = 0u; i < bus.joint_count; ++i) {
        if (bus.joints[i].node_id == node_id) {
            if (index_out != nullptr) *index_out = i;
            if (name_out != nullptr) *name_out = bus.joints[i].name;
            return true;
        }
    }
    return false;
}

const char *yes_no(bool v) noexcept { return v ? "yes" : "no"; }

}  // namespace

Status identify_bus(BusRuntime &bus, const IdentifyOptions &opt, IdentifyReport *out,
                    Result *res) noexcept
{
    if (out == nullptr) {
        if (res != nullptr) res->set(Status::kInvalidArgument, Advice::kNone, "identify: out is null");
        return Status::kInvalidArgument;
    }
    *out = IdentifyReport{};
    out->text[0] = '\0';

    jsdk_context *ctx = bus.raw_context();
    if (ctx == nullptr) {
        if (res != nullptr) {
            res->set(Status::kInvalidState, Advice::kNone,
                     "identify: bus '%s' is not open — open it first (the lock/ABI/HAL/zero-init "
                     "context setup belongs to BusRuntime::open())",
                     bus.cfg().name);
        }
        return Status::kInvalidState;
    }

    const BusCfg &bcfg = bus.cfg();

    /* ---- ① 发现（被动 200 ms + 主动探测 1..probe_max） ---- */
    std::uint8_t ids[kMaxJointsPerBus] = {};
    unsigned found = 0u;
    const jsdk_status_t dst =
        jsdk_context_discover(ctx, ids, static_cast<unsigned>(kMaxJointsPerBus), &found, opt.probe_max);
    if (dst != JSDK_OK) {
        const Status st = internal::map_sdk_status(dst);
        if (res != nullptr) {
            if (dst == JSDK_ERR_BAD_STATE) {
                /* SDK 的原话是 "discover() refused while a joint is enabled"。
                   现场最常见的原因不是"我们忘了失能"，而是**另一个 master 还在跑**
                   （节点进程/jsdk-cli）——两者要分开说，否则用户会去查错的方向。 */
                res->set(st, Advice::kEnsureSingleMaster,
                         "discover refused: %s. A joint is enabled on this context — make sure this "
                         "process is the only master and no joint is enabled (stop the `jr_bus` node "
                         "or `jsdk-cli`, or use the running node's services instead)",
                         jsdk_context_last_error(ctx));
            } else {
                res->set(st, internal::advice_from_sdk_text(jsdk_context_last_error(ctx)),
                         "discover failed: %s (%s)", jsdk_status_string(dst),
                         jsdk_context_last_error(ctx));
            }
        }
        return st;
    }
    out->count = found;
    add_line(out->text, sizeof(out->text), "[info] probe_max=%u -> discovered %u node(s)",
             static_cast<unsigned>(opt.probe_max), found);

    /* ⚠ 为什么只在 `read_config` 时才**加关节**：
       `BusRuntime` 的关节计数只由 `open()` 增加；在这里偷偷 `add_joint` 会让它
       自己的 `configure()` 看到 `joint_count_==0` —— 于是“识别阶段量程读到了、
       紧接着 configure 却说一个关节都没有”，症状是量程全 0 而不报错。
       所以：**只发现**（`read_config=false`）时绝不碰 context；
       要读设备（`read_config=true`，即“配置还没有、先问总线”的场景）时才加。 */
    jsdk_joint *handles[kMaxJointsPerBus] = {};
    if (opt.read_config) {
        for (unsigned k = 0u; k < found; ++k) {
            unsigned cfg_index = 0u;
            const char *cfg_name = nullptr;
            if (find_in_config(bcfg, ids[k], &cfg_index, &cfg_name)) {
                handles[k] = bus.raw_joint(cfg_index);   /* 复用（重复 add 会被 SDC 拒：node_id 冲突） */
            } else {
                jsdk_joint_config_t jc;
                std::memset(&jc, 0, sizeof jc);
                jc.magic = 0u;
                jc.node_id = ids[k];
                jc.axis = 0u;
                /* 只是为了让 per-joint API 有句柄；**不使能、不写任何东西**。
                   `initial_mode` 与“设备现在是什么模式”无关，也不影响只读路径。 */
                jc.initial_mode = JSDK_MODE_MIT;
                jsdk_joint_t *jt = nullptr;
                const jsdk_status_t ast = jsdk_context_add_joint(ctx, &jc, &jt);
                if (ast != JSDK_OK) {
                    if (res != nullptr) {
                        res->set(internal::map_sdk_status(ast), Advice::kNone,
                                 "add_joint(node %u) failed: %s (the bus may need a rebuild with a "
                                 "larger JSDK_MAX_JOINTS_STATIC)",
                                 static_cast<unsigned>(ids[k]), jsdk_status_string(ast));
                    }
                    return internal::map_sdk_status(ast);
                }
                handles[k] = jt;
            }
        }
    }

    /* 配置归属（两个分支都要报：调用方靠它判断“多出来的设备”） */
    for (unsigned k = 0u; k < found; ++k) {
        out->node[k].node_id = ids[k];
        unsigned cfg_index = 0u;
        const char *cfg_name = nullptr;
        if (find_in_config(bcfg, ids[k], &cfg_index, &cfg_name)) {
            out->node[k].in_config = true;
            if (cfg_name != nullptr) {
                std::snprintf(out->node[k].name, sizeof(out->node[k].name), "%s", cfg_name);
            }
        }
    }

    /* ---- ③ 设备身份（一帧 QUERY_DEVICE_INFO；不需要描述符） ---- */
    for (unsigned k = 0u; k < found; ++k) {
        if (handles[k] == nullptr) continue;
        jsdk_device_info_t di;
        std::memset(&di, 0, sizeof di);
        if (jsdk_joint_get_device_info(handles[k], &di) == JSDK_OK) {
            out->node[k].device.hw_version = di.hw_version;
            out->node[k].device.fw_version = di.fw_version;
            out->node[k].device.serial = di.serial;
            out->node[k].device.classic = (di.classic != 0u);
            out->node[k].device.valid = true;
        }
    }

    /* ---- ④ 可选：configure()（描述符 + 标定）——量程只能这样读到 ---- */
    Status result_st = Status::kOk;
    if (opt.read_config) {
        const jsdk_status_t cst = jsdk_context_configure(ctx);
        if (cst == JSDK_OK) {
            out->config_read = true;
        } else {
            /* ⚠ 不把失败说成成功，但**已经拿到的身份信息照给**：
               真实设备常出现"某个关节未标定 → configure 整条失败"，
               这时"谁在线、固件是什么"仍然是有效结论。 */
            result_st = internal::map_sdk_status(cst);
            if (res != nullptr) {
                res->set(result_st, internal::advice_from_sdk_text(jsdk_context_last_error(ctx)),
                         "configure() failed: %s (%s) — device identity below is still valid, but "
                         "ranges/calibration were NOT read",
                         jsdk_status_string(cst), jsdk_context_last_error(ctx));
            }
            add_line(out->text, sizeof(out->text),
                     "[warn] configure() failed: %s (`%s`) -> ranges/calibration NOT read",
                     jsdk_status_string(cst), jsdk_context_last_error(ctx));
        }
    }

    /* ---- ⑤ 逐节点：量程/心跳/超时/标定（读的是 configure 填好的本地缓存） ----
       ⚠ 整段只在 `read_config=true` 时跑：`read_config=false` 时我们**故意没读**设备，
         此时打 "device info read failed" 会把"没问"说成"问不到"（假告警）。 */
    for (unsigned k = 0u; opt.read_config && k < found; ++k) {
        if (handles[k] == nullptr) continue;
        jsdk_joint_config_snapshot_t snap;
        std::memset(&snap, 0, sizeof snap);
        if (jsdk_joint_read_config_snapshot(handles[k], &snap) == JSDK_OK) {
            JointInfoPOD &j = out->node[k].config;
            j.gear_ratio = snap.gear_ratio;
            j.mit_max_pos = snap.mit_max_pos;
            j.mit_max_vel = snap.mit_max_vel;
            j.mit_max_torque = snap.mit_max_torque;
            j.mit_max_kp = snap.mit_max_kp;
            j.mit_max_kd = snap.mit_max_kd;
            j.torque_constant = snap.torque_constant;
            j.heartbeat_rate_ms = snap.heartbeat_rate_ms;
            j.break_timeout_ms = snap.break_timeout_ms;
            j.node_id = snap.node_id;
            j.config_valid = (snap.valid != 0u);
            j.calibrated = (snap.valid != 0u);
        }

        const IdentifyNode &n = out->node[k];
        if (n.device.valid) {
            add_line(out->text, sizeof(out->text),
                     "[info] node %u%s%s%s: fw=0x%08x hw=0x%08x serial=0x%llx classic=%s",
                     static_cast<unsigned>(n.node_id), n.in_config ? " (" : "",
                     n.in_config ? n.name : "", n.in_config ? ")" : "",
                     n.device.fw_version, n.device.hw_version,
                     static_cast<unsigned long long>(n.device.serial), yes_no(n.device.classic));
        } else {
            add_line(out->text, sizeof(out->text),
                     "[warn] node %u answered discovery but device info read failed",
                     static_cast<unsigned>(n.node_id));
        }
        if (out->config_read) {
            add_line(out->text, sizeof(out->text),
                     "[info]   gear=%.4f pos=%.4f vel=%.4f trq=%.4f kp_max=%.1f kd_max=%.1f "
                     "hb=%u ms break_timeout=%u ms calibrated=%s",
                     n.config.gear_ratio, n.config.mit_max_pos, n.config.mit_max_vel,
                     n.config.mit_max_torque, n.config.mit_max_kp, n.config.mit_max_kd,
                     static_cast<unsigned>(n.config.heartbeat_rate_ms),
                     static_cast<unsigned>(n.config.break_timeout_ms),
                     yes_no(n.config.calibrated));
        }
    }

    /* ---- ⑥ 链路与描述符（原始计数/标志，供工具判断，不在这里下结论） ---- */
    jsdk_bus_state_t bs;
    std::memset(&bs, 0, sizeof bs);
    if (jsdk_context_get_bus_state(ctx, &bs) == JSDK_OK) {
        out->nodes_online = bs.nodes_online;
        out->link_up = (bs.link_up != 0u);
    }
    if (out->config_read) {
        /* `jsdk_desc_info_t` 与我们自己的 POD 字段不同名（SDK 是 `json_bytes`，我们是捕获字节数），
           所以逐字段拷（**不 reinterpret_cast**：结构体布局不受我们控制，硬转是等着被 ABI 改坏）。 */
        jsdk_desc_info_t dinfo;
        std::memset(&dinfo, 0, sizeof dinfo);
        if (jsdk_context_get_desc_info(ctx, &dinfo) == JSDK_OK) {
            out->desc.total_len = dinfo.total_len;
            out->desc.crc = dinfo.crc;
            out->desc.fw_version = dinfo.fw_version;
            out->desc.endpoint_count = dinfo.endpoint_count;
            out->desc.frames_rx = dinfo.frames_rx;
            out->desc.complete = (dinfo.complete != 0u);
            out->desc.shared_hit = (dinfo.shared_hit != 0u);
        }
    }
    add_line(out->text, sizeof(out->text), "[info] link_up=%s nodes_online=%u descriptor=%s",
             yes_no(out->link_up), static_cast<unsigned>(out->nodes_online),
             out->config_read ? "read" : "not read (read_config=false)");

    if (res != nullptr && result_st == Status::kOk) {
        res->set(Status::kOk, Advice::kNone, "identified %u node(s) on bus '%s'", found, bcfg.name);
    }
    return result_st;
}

}  // namespace rt
}  // namespace jr
