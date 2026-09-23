/**
 * @file    jr_bus_ops.cpp
 * @brief   BusRuntime 的运维面：参数读写、端点枚举、标定/回零/复位/点动
 *
 * @par 这一层只做三件事
 *  ① **守卫**：一切运维操作只允许在 READY / PAUSED 下执行（使能时做这些会把设备喂狗的
 *     控制帧挤掉 → 触发 disarm，这是 SDK 明确禁止的）；
 *  ② **不猜**：写参数前先查描述符，按**端点声明的类型**装箱；类型不符/超值域/路径不存在
 *     一律拒绝并说清原因（绝不静默截断，也绝不做"近似匹配"）；
 *  ③ **写后读回**：`requested` / `value`（读回）/ `verified` 三个字段分开给，
 *     读不回来就明说"无法校验"，绝不用请求值冒充结果。
 */

#include "jr_ros2/rt/jr_bus_runtime.hpp"

#include <cstdio>
#include <cstring>
#include <chrono>
#include <thread>

#include "joint_sdk/joint_sdk.h"

#include "jr_ros2/rt/jr_rt_sched.hpp"
#include "jr_sdk_map.hpp"

namespace jr {
namespace rt {
namespace {

/* 共享的状态码/建议映射（单一映射点，见 jr_sdk_map.hpp） */
using internal::advice_from_sdk_text;
using internal::map_sdk_status;
/** SDK 单次批量读的上限是 31 个端点；这里取 16，保证帧与栈都留有余量。 */
constexpr unsigned kMaxParamBatch = 16u;

ParamType type_from_sdk(jsdk_ep_type_t t) noexcept
{
    switch (t) {
    case JSDK_EP_BOOL: return ParamType::kBool;
    case JSDK_EP_U8:   return ParamType::kU8;
    case JSDK_EP_I8:   return ParamType::kI8;
    case JSDK_EP_U16:  return ParamType::kU16;
    case JSDK_EP_I16:  return ParamType::kI16;
    case JSDK_EP_U32:  return ParamType::kU32;
    case JSDK_EP_I32:  return ParamType::kI32;
    case JSDK_EP_U64:  return ParamType::kU64;
    case JSDK_EP_I64:  return ParamType::kI64;
    case JSDK_EP_F32:  return ParamType::kF32;
    case JSDK_EP_F64:  return ParamType::kF64;
    default:           return ParamType::kUnsupported;   /* object/json/ref/function */
    }
}

jsdk_ep_type_t type_to_sdk(ParamType t) noexcept
{
    switch (t) {
    case ParamType::kBool: return JSDK_EP_BOOL;
    case ParamType::kU8:   return JSDK_EP_U8;
    case ParamType::kI8:   return JSDK_EP_I8;
    case ParamType::kU16:  return JSDK_EP_U16;
    case ParamType::kI16:  return JSDK_EP_I16;
    case ParamType::kU32:  return JSDK_EP_U32;
    case ParamType::kI32:  return JSDK_EP_I32;
    case ParamType::kU64:  return JSDK_EP_U64;
    case ParamType::kI64:  return JSDK_EP_I64;
    case ParamType::kF32:  return JSDK_EP_F32;
    case ParamType::kF64:  return JSDK_EP_F64;
    case ParamType::kUnsupported: break;
    }
    return JSDK_EP_U32;   /* 不会被调用（调用点已拒绝 Unsupported） */
}

jsdk_value_t value_to_sdk(const ParamValue &v) noexcept
{
    jsdk_value_t out;
    std::memset(&out, 0, sizeof out);
    out.type = type_to_sdk(v.type);
    switch (v.type) {
    case ParamType::kBool: out.v.boolean = v.v.b ? 1 : 0; break;
    case ParamType::kU8:   out.v.u8 = v.v.u8; break;
    case ParamType::kI8:   out.v.i8 = v.v.i8; break;
    case ParamType::kU16:  out.v.u16 = v.v.u16; break;
    case ParamType::kI16:  out.v.i16 = v.v.i16; break;
    case ParamType::kU32:  out.v.u32 = v.v.u32; break;
    case ParamType::kI32:  out.v.i32 = v.v.i32; break;
    case ParamType::kU64:  out.v.u64 = v.v.u64; break;
    case ParamType::kI64:  out.v.i64 = v.v.i64; break;
    case ParamType::kF32:  out.v.f32 = v.v.f32; break;
    case ParamType::kF64:  out.v.f64 = v.v.f64; break;
    case ParamType::kUnsupported: break;
    }
    return out;
}

ParamValue value_from_sdk(const jsdk_value_t &v) noexcept
{
    ParamValue out;
    out.type = type_from_sdk(v.type);
    switch (out.type) {
    case ParamType::kBool: out.v.b = (v.v.boolean != 0); break;
    case ParamType::kU8:   out.v.u8 = v.v.u8; break;
    case ParamType::kI8:   out.v.i8 = v.v.i8; break;
    case ParamType::kU16:  out.v.u16 = v.v.u16; break;
    case ParamType::kI16:  out.v.i16 = v.v.i16; break;
    case ParamType::kU32:  out.v.u32 = v.v.u32; break;
    case ParamType::kI32:  out.v.i32 = v.v.i32; break;
    case ParamType::kU64:  out.v.u64 = v.v.u64; break;
    case ParamType::kI64:  out.v.i64 = v.v.i64; break;
    case ParamType::kF32:  out.v.f32 = v.v.f32; break;
    case ParamType::kF64:  out.v.f64 = v.v.f64; break;
    case ParamType::kUnsupported: break;
    }
    return out;
}

Status map_sdk(jsdk_status_t st) noexcept { return internal::map_sdk_status(st); }

void copy_str(char *dst, std::size_t cap, const char *src) noexcept
{
    if (dst == nullptr || cap == 0u) return;
    std::snprintf(dst, cap, "%s", (src != nullptr) ? src : "");
}

/**
 * 端点过滤匹配（与 CLI `ep-list --filter` 的语义一致，并写进头文件）：
 * 空 = 全匹配；`*` 结尾 = 前缀；`.` 结尾 = 段前缀；否则 = 子串。
 */
bool filter_match(const char *path, const char *filter) noexcept
{
    if (filter == nullptr || filter[0] == '\0') return true;
    const std::size_t flen = std::strlen(filter);
    if (filter[flen - 1u] == '*') {
        return std::strncmp(path, filter, flen - 1u) == 0;
    }
    if (filter[flen - 1u] == '.') {
        return std::strncmp(path, filter, flen) == 0;
    }
    return std::strstr(path, filter) != nullptr;
}

struct ListCtx {
    const char   *filter = nullptr;
    EndpointInfo *out = nullptr;
    unsigned      cap = 0u;
    unsigned      count = 0u;
};

/**
 * "写进去就被固件立即消费"的端点：读回与请求**必然不同**，但这不是失败。
 *
 * 依据：固件在 `axis.cpp` 里拿到 `requested_state` 后立刻把它复位为 UNDEFINED，
 * 状态机去执行动作。把它们当成"写失败"或"未校验失败"，客户会以为标定/回零坏了。
 * 目前只有这一条已知（要与 SDK 的 CLI 行为保持一致）。
 */
bool is_firmware_consumed_path(const char *path) noexcept
{
    return (path != nullptr) &&
           (std::strstr(path, "requested_state") != nullptr);
}

int list_visit(void *user, const char *path, std::uint16_t id, jsdk_ep_type_t type,
               std::uint8_t access) noexcept
{
    auto *ctx = static_cast<ListCtx *>(user);
    if (ctx == nullptr || path == nullptr) return 1;
    if (!filter_match(path, ctx->filter)) return 0;   /* 继续遍历 */
    if (ctx->count < ctx->cap) {
        EndpointInfo &e = ctx->out[ctx->count];
        copy_str(e.path, sizeof(e.path), path);
        e.id = id;
        e.type = type_from_sdk(type);
        e.access = access;
    }
    ++ctx->count;
    return 0;
}

}  // namespace

/* ==========================================================================
 * 守卫
 * ======================================================================== */

bool BusRuntime::ops_window_ok(Result *res, const char *what) noexcept
{
    if (ctx_ == nullptr) {
        if (res != nullptr) res->set(Status::kInvalidState, Advice::kNone, "bus '%s' is not open", bus_.name);
        return false;
    }
    if (mode_ == BusMode::kActive) {
        if (res != nullptr) {
            res->set(Status::kInvalidState, Advice::kNone,
                     "bus '%s' is ACTIVE: '%s' is not allowed while joints are enabled — parameter / "
                     "descriptor frames would starve the control frames and trip the device "
                     "watchdog. Pause the tick group first (the node does this automatically; from "
                     "the CLI it means: stop sending commands, then use the service).",
                     bus_.name, what);
        }
        return false;
    }
    if (mode_ == BusMode::kIdle) {
        if (res != nullptr) {
            res->set(Status::kInvalidState, Advice::kNone,
                     "bus '%s' is not configured yet: '%s' needs the endpoint table (descriptor). "
                     "Call configure() first.",
                     bus_.name, what);
        }
        return false;
    }
    return true;
}

bool BusRuntime::select_joints(const unsigned *idx, unsigned n, unsigned *out, unsigned *count_out,
                              Result *res) const noexcept
{
    if (idx == nullptr || n == 0u) {
        for (unsigned i = 0u; i < joint_count_; ++i) out[i] = i;
        *count_out = joint_count_;
        return true;
    }
    if (n > kMaxJointsPerBus) {
        if (res != nullptr) {
            res->set(Status::kInvalidArgument, Advice::kNone,
                     "too many joints selected (%u > %u)", n, kMaxJointsPerBus);
        }
        return false;
    }
    for (unsigned i = 0u; i < n; ++i) {
        if (idx[i] >= joint_count_) {
            if (res != nullptr) {
                res->set(Status::kInvalidArgument, Advice::kNone,
                         "joint index %u out of range (bus '%s' has %u joint(s))", idx[i], bus_.name,
                         joint_count_);
            }
            return false;
        }
        out[i] = idx[i];
    }
    *count_out = n;
    return true;
}

/* ==========================================================================
 * 参数读
 * ======================================================================== */

Status BusRuntime::read_params(ParamReadItem *items, unsigned count, Result *res) noexcept
{
    if (items == nullptr || count == 0u) {
        if (res != nullptr) res->set(Status::kInvalidArgument, Advice::kNone, "no item to read");
        return Status::kInvalidArgument;
    }
    if (!ops_window_ok(res, "read_params")) return Status::kInvalidState;

    unsigned ok_count = 0u;
    unsigned bad_count = 0u;

    /* 先处理 joint_index 非法的项（保持"逐项有结论"的契约）。 */
    for (unsigned i = 0u; i < count; ++i) {
        ParamReadItem &it = items[i];
        it.value = ParamValue{};
        it.declared_type = ParamType::kUnsupported;
        it.message[0] = '\0';
        if (it.joint_index >= joint_count_) {
            it.status = Status::kInvalidArgument;
            std::snprintf(it.message, sizeof(it.message), "joint_index %u out of range (0..%u)",
                          it.joint_index, joint_count_ - 1u);
            ++bad_count;
            continue;
        }
        if (it.path == nullptr || it.path[0] == '\0') {
            it.status = Status::kInvalidArgument;
            copy_str(it.message, sizeof(it.message), "empty path");
            ++bad_count;
        }
    }

    for (unsigned j = 0u; j < joint_count_; ++j) {
        /* 同一关节的项分批走 SDK 的批量读（FD 下会打进一帧）。 */
        unsigned batch = 0u;
        unsigned idx[kMaxParamBatch] = {};
        jsdk_param_req_t reqs[kMaxParamBatch];

        for (unsigned i = 0u; i < count; ++i) {
            ParamReadItem &it = items[i];
            if (it.joint_index != j || it.path == nullptr || it.path[0] == '\0') continue;

            std::memset(&reqs[batch], 0, sizeof(reqs[batch]));
            reqs[batch].path = it.path;
            idx[batch] = i;
            ++batch;

            if (batch == kMaxParamBatch) {
                (void)jsdk_joint_param_get_batch(joints_[j], reqs, batch);
                for (unsigned k = 0u; k < batch; ++k) {
                    ParamReadItem &tgt = items[idx[k]];
                    tgt.value = value_from_sdk(reqs[k].value);
                    tgt.status = map_sdk(reqs[k].status);
                    if (tgt.status == Status::kOk) {
                        ++ok_count;
                    } else {
                        ++bad_count;
                        copy_str(tgt.message, sizeof(tgt.message), jsdk_status_string(reqs[k].status));
                    }
                }
                batch = 0u;
            }
        }
        if (batch > 0u) {
            (void)jsdk_joint_param_get_batch(joints_[j], reqs, batch);
            for (unsigned k = 0u; k < batch; ++k) {
                ParamReadItem &tgt = items[idx[k]];
                tgt.value = value_from_sdk(reqs[k].value);
                tgt.status = map_sdk(reqs[k].status);
                if (tgt.status == Status::kOk) {
                    ++ok_count;
                } else {
                    ++bad_count;
                    copy_str(tgt.message, sizeof(tgt.message), jsdk_status_string(reqs[k].status));
                }
            }
        }
    }

    /* 补上声明类型（描述符查询，便于上层显示"这是 uint16 端点"）。 */
    for (unsigned i = 0u; i < count; ++i) {
        ParamReadItem &it = items[i];
        if (it.status == Status::kOk) {
            std::uint16_t ep = 0u;
            jsdk_ep_type_t t = JSDK_EP_U32;
            std::uint8_t acc = 0u;
            if (jsdk_endpoint_lookup(ctx_, it.path, &ep, &t, &acc) == JSDK_OK) {
                it.declared_type = type_from_sdk(t);
            }
        }
    }

    const Status overall = (bad_count == 0u) ? Status::kOk : Status::kProtocol;
    if (res != nullptr) {
        res->set(overall, (overall == Status::kOk) ? Advice::kNone : Advice::kCheckBusConfig,
                 "read %u item(s): %u ok, %u failed (per-item reasons in the response)",
                 count, ok_count, bad_count);
    }
    return overall;
}

/* ==========================================================================
 * 参数写（写后读回）
 * ======================================================================== */

Status BusRuntime::write_params(ParamWriteItem *items, unsigned count, bool confirm, bool persist,
                                Result *res) noexcept
{
    if (items == nullptr || count == 0u) {
        if (res != nullptr) res->set(Status::kInvalidArgument, Advice::kNone, "no item to write");
        return Status::kInvalidArgument;
    }
    if (!confirm) {
        if (res != nullptr) {
            res->set(Status::kInvalidState, Advice::kNone,
                     "refused: %u write request(s) without confirm=true. Writes change device "
                     "behaviour (watchdog/heartbeat/limits) and are gated on an explicit "
                     "confirmation — pass confirm=true when you really mean it.",
                     count);
        }
        return Status::kInvalidState;
    }
    if (!ops_window_ok(res, "write_params")) return Status::kInvalidState;

    unsigned ok_count = 0u;
    unsigned unverified_count = 0u;
    unsigned failed_count = 0u;

    for (unsigned i = 0u; i < count; ++i) {
        ParamWriteItem &it = items[i];
        it.status = Status::kOk;
        it.verified = false;
        it.consumed_by_firmware = false;
        it.persisted = false;
        it.declared_type = ParamType::kUnsupported;
        it.value = ParamValue{};
        it.message[0] = '\0';

        if (it.joint_index >= joint_count_) {
            it.status = Status::kInvalidArgument;
            std::snprintf(it.message, sizeof(it.message), "joint_index %u out of range (0..%u)",
                          it.joint_index, joint_count_ - 1u);
            ++failed_count;
            continue;
        }
        if (it.path == nullptr || it.path[0] == '\0') {
            it.status = Status::kInvalidArgument;
            copy_str(it.message, sizeof(it.message), "empty path");
            ++failed_count;
            continue;
        }

        /* ① 先查描述符：拿声明类型与权限（**不猜**）。 */
        std::uint16_t ep = 0u;
        jsdk_ep_type_t sdk_type = JSDK_EP_U32;
        std::uint8_t access = 0u;
        const jsdk_status_t lst = jsdk_endpoint_lookup(ctx_, it.path, &ep, &sdk_type, &access);
        if (lst != JSDK_OK) {
            it.status = Status::kNotFound;
            std::snprintf(it.message, sizeof(it.message),
                          "path '%s' does not exist in this device's descriptor (we do not do "
                          "prefix/approximate matching); use list_endpoints to find the real path",
                          it.path);
            ++failed_count;
            continue;
        }
        it.declared_type = type_from_sdk(sdk_type);
        if (it.declared_type == ParamType::kUnsupported) {
            it.status = Status::kNotSupported;
            std::snprintf(it.message, sizeof(it.message), "endpoint type is not a writable scalar (%s)",
                          param_type_name(it.declared_type));
            ++failed_count;
            continue;
        }
        if ((access & 0x02u) == 0u) {
            it.status = Status::kNotSupported;
            copy_str(it.message, sizeof(it.message), "endpoint is read-only");
            ++failed_count;
            continue;
        }
        if (it.requested.type != it.declared_type) {
            /* 这条检查是"先查描述符按类型装箱"的落地：类型不符会被设备拒绝（C 侧报
               descriptor=uint16 given=uint32），与其让设备报错，不如在这里说清楚。 */
            it.status = Status::kInvalidArgument;
            std::snprintf(it.message, sizeof(it.message),
                          "type mismatch: endpoint declares %s but the request carries %s "
                          "(parse the text with the declared type: parse_param_text)",
                          param_type_name(it.declared_type), param_type_name(it.requested.type));
            ++failed_count;
            continue;
        }

        /* ② 写。 */
        const jsdk_value_t jv = value_to_sdk(it.requested);
        const jsdk_status_t wst = jsdk_joint_param_set(joints_[it.joint_index], it.path, &jv);
        if (wst != JSDK_OK) {
            it.status = map_sdk(wst);
            std::snprintf(it.message, sizeof(it.message), "device/transport rejected the write: %s (%s)",
                          jsdk_status_string(wst), jsdk_context_last_error(ctx_));
            ++failed_count;
            continue;
        }

        /* ③ 读回校验：不可读回时**明确报告无法校验**（不是失败，也不是成功）。 */
        jsdk_value_t rb;
        std::memset(&rb, 0, sizeof rb);
        const jsdk_status_t rst = jsdk_joint_param_get(joints_[it.joint_index], it.path, &rb);
        if (rst != JSDK_OK) {
            it.status = Status::kUnverified;
            std::snprintf(it.message, sizeof(it.message),
                          "write was accepted but could NOT be read back (%s): cannot verify. "
                          "Known case: axis0.config.can.break_timeout reads back as 0 on current "
                          "firmware (F28).",
                          jsdk_status_string(rst));
            ++unverified_count;
            continue;
        }

        it.value = value_from_sdk(rb);
        if (param_values_equal(it.value, it.requested)) {
            it.verified = true;
            it.status = Status::kOk;
            copy_str(it.message, sizeof(it.message), "verified by read-back");
            ++ok_count;
        } else if (is_firmware_consumed_path(it.path)) {
            /* 读回不同是**预期行为**：固件把值收走了。不算失败，但也不声称已校验。
               文本里**不重复路径**（it.path 已带，且 160 B 的 message 装不下
               145 B 固定文字 + 128 B 路径 —— GCC `-O2` 会直接以 -Wformat-truncation 报错）。 */
            it.consumed_by_firmware = true;
            it.status = Status::kOk;
            std::snprintf(it.message, sizeof(it.message),
                          "accepted; the firmware consumes this endpoint immediately (state "
                          "machine), so the read-back differs by design - NOT a failure. Check "
                          "the resulting state.");
            ++ok_count;
        } else {
            char want[48] = {};
            char got[48] = {};
            format_param_value(it.requested, want, sizeof(want));
            format_param_value(it.value, got, sizeof(got));
            it.status = Status::kUnverified;
            std::snprintf(it.message, sizeof(it.message),
                          "read-back differs: requested %s, device reports %s", want, got);
            ++unverified_count;
        }
    }

    /* ④ 持久化（只有"全部被接受"才落 Flash，避免写了一半就存进 Flash）。 */
    if (persist && failed_count == 0u && count > 0u) {
        Result save_res;
        const Status sst = save_config(&save_res);
        if (sst == Status::kOk) {
            /* 只给**已校验/已确认被接受**的项打 persisted。
               （自审发现：原先给所有项都打了 true —— 对"读回不一致"的项，
                我们并不知道落进 Flash 的是不是请求值，那就是在说"比知道的更多"。） */
            for (unsigned i = 0u; i < count; ++i) {
                if (items[i].status == Status::kOk) items[i].persisted = true;
            }
        } else if (res != nullptr) {
            res->set(sst, save_res.advice,
                     "parameters were written but CONFIG_SAVE failed: %s (values are live but will "
                     "be lost on power cycle)",
                     save_res.message);
            return sst;
        }
    }

    Status overall = Status::kOk;
    if (failed_count > 0u) overall = Status::kProtocol;
    else if (unverified_count > 0u) overall = Status::kUnverified;   /* ← 绝不当成功 */

    if (res != nullptr) {
        res->set(overall, (overall == Status::kOk) ? Advice::kNone : Advice::kCheckBusConfig,
                 "wrote %u item(s): %u verified, %u unverified, %u failed%s", count, ok_count,
                 unverified_count, failed_count, persist ? " (persisted to flash)" : "");
    }
    return overall;
}

/* ==========================================================================
 * 端点
 * ======================================================================== */

Status BusRuntime::lookup_endpoint(const char *path, EndpointInfo *out, Result *res) noexcept
{
    if (path == nullptr || out == nullptr) {
        if (res != nullptr) res->set(Status::kInvalidArgument, Advice::kNone, "null argument");
        return Status::kInvalidArgument;
    }
    if (!ops_window_ok(res, "lookup_endpoint")) return Status::kInvalidState;

    *out = EndpointInfo{};
    std::uint16_t ep = 0u;
    jsdk_ep_type_t t = JSDK_EP_U32;
    std::uint8_t acc = 0u;
    const jsdk_status_t st = jsdk_endpoint_lookup(ctx_, path, &ep, &t, &acc);
    if (st != JSDK_OK) {
        if (res != nullptr) {
            res->set(Status::kNotFound, Advice::kNone,
                     "endpoint '%s' not found in this device (no approximate matching; try "
                     "list_endpoints with a substring filter)",
                     path);
        }
        return Status::kNotFound;
    }
    copy_str(out->path, sizeof(out->path), path);
    out->id = ep;
    out->type = type_from_sdk(t);
    out->access = acc;
    if (res != nullptr) {
        res->set(Status::kOk, Advice::kNone, "endpoint '%s': id=%u type=%s access=%s%s", path,
                 static_cast<unsigned>(ep), param_type_name(out->type), out->readable() ? "r" : "-",
                 out->writable() ? "w" : "-");
    }
    return Status::kOk;
}

Status BusRuntime::list_endpoints(const char *filter, EndpointInfo *out, unsigned cap,
                                  unsigned *count_out, Result *res) noexcept
{
    if (out == nullptr || cap == 0u) {
        if (res != nullptr) res->set(Status::kInvalidArgument, Advice::kNone, "empty output buffer");
        return Status::kInvalidArgument;
    }
    if (!ops_window_ok(res, "list_endpoints")) return Status::kInvalidState;

    ListCtx ctx;
    ctx.filter = filter;
    ctx.out = out;
    ctx.cap = cap;
    for (unsigned i = 0u; i < cap; ++i) out[i] = EndpointInfo{};

    const jsdk_status_t st = jsdk_endpoint_enumerate(ctx_, &list_visit, &ctx);
    if (count_out != nullptr) *count_out = ctx.count;

    if (st != JSDK_OK) {
        if (res != nullptr) {
            res->set(map_sdk(st), Advice::kNone, "endpoint enumeration failed: %s",
                     jsdk_status_string(st));
        }
        return map_sdk(st);
    }
    if (res != nullptr) {
        res->set(Status::kOk, Advice::kNone, "matched %u endpoint(s)%s (returned %u, cap %u)",
                 ctx.count, (filter != nullptr && filter[0] != '\0') ? " for the given filter" : "",
                 (ctx.count < cap) ? ctx.count : cap, cap);
    }
    return Status::kOk;
}

/* ==========================================================================
 * 运维
 * ======================================================================== */

Status BusRuntime::set_zero_here(Result *res) noexcept
{
    return set_zero_here(nullptr, 0u, res);
}

Status BusRuntime::set_zero_here(const unsigned *idx, unsigned n, Result *res) noexcept
{
    if (!ops_window_ok(res, "set_zero")) return Status::kInvalidState;
    const unsigned count = (idx == nullptr || n == 0u) ? joint_count_ : n;
    for (unsigned k = 0u; k < count; ++k) {
        const unsigned j = (idx == nullptr || n == 0u) ? k : idx[k];
        if (j >= joint_count_) {
            if (res != nullptr) {
                res->set(Status::kInvalidArgument, Advice::kNone, "joint_index %u out of range (0..%u)",
                         j, joint_count_ - 1u);
            }
            return Status::kInvalidArgument;
        }
        const jsdk_status_t st = jsdk_joint_set_zero_here(joints_[j]);
        if (st != JSDK_OK) {
            const Status s = map_sdk(st);
            if (res != nullptr) {
                res->set(s, Advice::kNone,
                         "joint '%s': SET_ZERO failed: %s (%s). Note: this only sets the zero "
                         "offset in RAM — use save_config to persist.",
                         bus_.joints[j].name, jsdk_status_string(st), jsdk_context_last_error(ctx_));
            }
            return s;
        }
    }
    if (res != nullptr) {
        res->set(Status::kOk, Advice::kNone,
                 "zero set on %u joint(s). NOT persisted (call save_config for flash).", count);
    }
    return Status::kOk;
}

Status BusRuntime::calibrate(Result *res) noexcept
{
    return calibrate(nullptr, 0u, res);
}

Status BusRuntime::calibrate(const unsigned *idx, unsigned n, Result *res) noexcept
{
    if (!ops_window_ok(res, "calibrate")) return Status::kInvalidState;
    unsigned sel[kMaxJointsPerBus] = {};
    unsigned sel_count = 0u;
    if (!select_joints(idx, n, sel, &sel_count, res)) return Status::kInvalidArgument;

    for (unsigned k = 0u; k < sel_count; ++k) {
        const unsigned j = sel[k];
        const jsdk_status_t st = jsdk_joint_calibrate(joints_[j]);
        if (st != JSDK_OK) {
            const Status s = map_sdk(st);
            if (res != nullptr) {
                res->set(s, advice_from_sdk_text(jsdk_context_last_error(ctx_)),
                         "joint '%s': calibration failed: %s — %s. The full sequence turns the "
                         "motor through many electrical revolutions and takes tens of seconds "
                         "(budget = BusCfg.state_timeout_ms, 0 = SDK default 120 s); make sure the "
                         "joint is mechanically free to move.",
                         bus_.joints[j].name, jsdk_status_string(st),
                         jsdk_context_last_error(ctx_));
            }
            return s;
        }
    }

    /* 状态跑完 ≠ 标定生效：必须**读回** pre_calibrated（SDK 文档明确要求）。 */
    bool verified = true;
    char detail[192] = {};
    for (unsigned k = 0u; k < sel_count; ++k) {
        const unsigned j = sel[k];
        int motor = 0;
        int enc = 0;
        const bool got_motor =
            (jsdk_joint_param_get_bool(joints_[j], "axis0.motor.config.pre_calibrated", &motor) == JSDK_OK);
        const bool got_enc =
            (jsdk_joint_param_get_bool(joints_[j], "axis0.encoder.config.pre_calibrated", &enc) == JSDK_OK);
        if (!got_motor || !got_enc || motor == 0 || enc == 0) {
            verified = false;
            std::snprintf(detail, sizeof(detail),
                          "joint '%s': motor_pre_calibrated=%s encoder_pre_calibrated=%s%s",
                          bus_.joints[j].name, got_motor ? (motor ? "true" : "false") : "unreadable",
                          got_enc ? (enc ? "true" : "false") : "unreadable",
                          (!got_motor || !got_enc) ? " (read-back failed)" : "");
            break;
        }
    }

    /* 标定会让量程生效 → 刷新报告（量程/标定标志/限位检查都会重算）。 */
    (void)read_back_after_configure(&report_, nullptr);

    if (!verified) {
        if (res != nullptr) {
            res->set(Status::kUnverified, Advice::kCalibrationRequired,
                     "calibration sequence finished but the pre_calibrated flags do NOT read back as "
                     "true: %s. Do not trust this calibration — re-run or check the firmware.",
                     detail);
        }
        return Status::kUnverified;
    }
    if (res != nullptr) {
        res->set(Status::kOk, Advice::kNone,
                 "calibration verified: motor & encoder pre_calibrated = true; calibrated range was "
                 "re-read");
    }
    return Status::kOk;
}

Status BusRuntime::home(Result *res) noexcept
{
    return home(nullptr, 0u, res);
}

Status BusRuntime::home(const unsigned *idx, unsigned n, Result *res) noexcept
{
    if (!ops_window_ok(res, "home")) return Status::kInvalidState;
    unsigned sel[kMaxJointsPerBus] = {};
    unsigned sel_count = 0u;
    if (!select_joints(idx, n, sel, &sel_count, res)) return Status::kInvalidArgument;
    for (unsigned k = 0u; k < sel_count; ++k) {
        const unsigned j = sel[k];
        const jsdk_status_t st = jsdk_joint_home(joints_[j]);
        if (st != JSDK_OK) {
            const Status s = map_sdk(st);
            if (res != nullptr) {
                res->set(s, advice_from_sdk_text(jsdk_context_last_error(ctx_)),
                         "joint '%s': homing failed: %s — %s", bus_.joints[j].name,
                         jsdk_status_string(st), jsdk_context_last_error(ctx_));
            }
            return s;
        }
    }
    if (res != nullptr) res->set(Status::kOk, Advice::kNone, "homing finished on %u joint(s)", sel_count);
    return Status::kOk;
}

Status BusRuntime::save_config(Result *res) noexcept
{
    return save_config(nullptr, 0u, res);
}

Status BusRuntime::save_config(const unsigned *idx, unsigned n, Result *res) noexcept
{
    if (ctx_ == nullptr) {
        if (res != nullptr) res->set(Status::kInvalidState, Advice::kNone, "bus '%s' is not open", bus_.name);
        return Status::kInvalidState;
    }
    if (mode_ == BusMode::kActive) {
        if (res != nullptr) {
            res->set(Status::kInvalidState, Advice::kNone,
                     "bus '%s' is ACTIVE: CONFIG_SAVE is blocked while joints are enabled (same "
                     "reason as parameter writes). Pause first.",
                     bus_.name);
        }
        return Status::kInvalidState;
    }
    unsigned sel[kMaxJointsPerBus] = {};
    unsigned sel_count = 0u;
    if (!select_joints(idx, n, sel, &sel_count, res)) return Status::kInvalidArgument;
    for (unsigned k = 0u; k < sel_count; ++k) {
        const unsigned j = sel[k];
        const jsdk_status_t st = jsdk_joint_save_config(joints_[j]);
        if (st != JSDK_OK) {
            const Status s = map_sdk(st);
            if (res != nullptr) {
                res->set(s, Advice::kNone, "joint '%s': CONFIG_SAVE failed: %s (%s)",
                         bus_.joints[j].name, jsdk_status_string(st), jsdk_context_last_error(ctx_));
            }
            return s;
        }
    }
    if (res != nullptr) res->set(Status::kOk, Advice::kNone, "configuration saved to flash (%u joint(s))", sel_count);
    return Status::kOk;
}

Status BusRuntime::reset_device(Result *res) noexcept
{
    return reset_device(nullptr, 0u, res);
}

Status BusRuntime::reset_device(const unsigned *idx, unsigned n, Result *res) noexcept
{
    if (!ops_window_ok(res, "reset_device")) return Status::kInvalidState;
    unsigned sel[kMaxJointsPerBus] = {};
    unsigned sel_count = 0u;
    if (!select_joints(idx, n, sel, &sel_count, res)) return Status::kInvalidArgument;
    for (unsigned k = 0u; k < sel_count; ++k) {
        const unsigned j = sel[k];
        const jsdk_status_t st = jsdk_joint_reset_device(joints_[j]);
        if (st != JSDK_OK) {
            const Status s = map_sdk(st);
            if (res != nullptr) {
                res->set(s, Advice::kNone, "joint '%s': RESET_DEVICE failed: %s (%s)",
                         bus_.joints[j].name, jsdk_status_string(st), jsdk_context_last_error(ctx_));
            }
            return s;
        }
    }
    /* 复位后设备重新上电自检：必须重新握手/configure（SDK 明确要求）。 */
    mode_ = BusMode::kIdle;
    note_append("[info] device reset requested: the bus returned to IDLE, run configure() again");
    if (res != nullptr) {
        res->set(Status::kOk, Advice::kNone,
                 "reset requested on %u joint(s). The bus is now IDLE: call configure() again before "
                 "any further operation (node id may revert to the flash value).",
                 sel_count);
    }
    return Status::kOk;
}

Status BusRuntime::set_node_id(unsigned local_index, std::uint8_t new_id, bool persist,
                               Result *res) noexcept
{
    if (local_index >= joint_count_) {
        if (res != nullptr) {
            res->set(Status::kInvalidArgument, Advice::kNone, "joint_index %u out of range (0..%u)",
                     local_index, joint_count_ - 1u);
        }
        return Status::kInvalidArgument;
    }
    if (!ops_window_ok(res, "set_node_id")) return Status::kInvalidState;
    if (new_id == 0u || new_id > 254u) {
        if (res != nullptr) {
            res->set(Status::kInvalidArgument, Advice::kCheckBusConfig,
                     "new node_id %u is invalid (must be 1..254)", static_cast<unsigned>(new_id));
        }
        return Status::kInvalidArgument;
    }

    const jsdk_status_t st = jsdk_joint_set_node_id(joints_[local_index], new_id, persist ? 1 : 0);
    if (st != JSDK_OK) {
        if (res != nullptr) {
            res->set(map_sdk(st), Advice::kCheckBusConfig,
                     "joint '%s': changing node_id to %u failed: %s — %s (the SDK refuses when the "
                     "target address already answers on the bus: that would create two devices with "
                     "the same id)",
                     bus_.joints[local_index].name, static_cast<unsigned>(new_id),
                     jsdk_status_string(st), jsdk_context_last_error(ctx_));
        }
        return map_sdk(st);
    }

    /* 自己的记账必须跟着改，否则快照/诊断会继续报旧地址。 */
    bus_.joints[local_index].node_id = new_id;
    note_append("[info] joint '%s' node_id -> %u%s", bus_.joints[local_index].name,
                static_cast<unsigned>(new_id), persist ? " (persisted)" : "");
    if (res != nullptr) {
        res->set(Status::kOk, Advice::kNone, "joint '%s' is now node %u%s",
                 bus_.joints[local_index].name, static_cast<unsigned>(new_id),
                 persist ? " (saved to flash)" : " (NOT persisted: use persist=true)");
    }
    return Status::kOk;
}

Status BusRuntime::jog(unsigned local_index, const JointTarget &target, unsigned duration_ms,
                       bool confirm, Result *res) noexcept
{
    if (local_index >= joint_count_) {
        if (res != nullptr) {
            res->set(Status::kInvalidArgument, Advice::kNone, "joint_index %u out of range (0..%u)",
                     local_index, joint_count_ - 1u);
        }
        return Status::kInvalidArgument;
    }
    if (!confirm) {
        if (res != nullptr) {
            res->set(Status::kInvalidState, Advice::kNone,
                     "refused: jog moves the motor, so it requires confirm=true (the same gate as "
                     "the CLI's --yes --hold)");
        }
        return Status::kInvalidState;
    }
    if (duration_ms == 0u || duration_ms > 10000u) {
        if (res != nullptr) {
            res->set(Status::kInvalidArgument, Advice::kNone,
                     "duration_ms=%u out of range: 1..10000 (hard limit 10 s — a jog must be "
                     "self-limiting)",
                     duration_ms);
        }
        return Status::kInvalidArgument;
    }
    if (!ops_window_ok(res, "jog")) return Status::kInvalidState;

    /* ⚠ Jog 是** MIT 语义**的动作（跑到一个位置并 PD 保持）：它内部用 MIT 使能 +
       `set_mit*()` 发帧。对一个配置成 CSP/CSV/CST/CURRENT 的关节做这件事，
       等于“做动作的同时把它的模式换回去”（等效于未受保护的模式切换）——
       所以直接拒，并说清怎么办（改成配置里的模式 + 重配置/重启）。 */
    if (joint_mode_[local_index] != static_cast<std::uint8_t>(CmdMode::kMit)) {
        if (res != nullptr) {
            res->set(Status::kInvalidState, Advice::kCheckBusConfig,
                     "joint '%s' is configured in '%s' mode: Jog is a MIT-mode action and would "
                     "silently switch the joint's input mode. Use the '~/cmd' topic with a target "
                     "in the joint's own mode, or set joints[].mode: mit in the config and "
                     "restart.",
                     bus_.joints[local_index].name,
                     to_string(static_cast<CmdMode>(joint_mode_[local_index])));
        }
        return Status::kInvalidState;
    }

    const JointInfoPOD &info = report_.joint[local_index];
    if (!info.calibrated) {
        if (res != nullptr) {
            res->set(Status::kNotCalibrated, Advice::kCalibrationRequired,
                     "joint '%s' is not calibrated: physical-quantity jog would be rejected by the "
                     "SDK anyway. Calibrate or set zero first.",
                     bus_.joints[local_index].name);
        }
        return Status::kNotCalibrated;
    }

    jsdk_joint_t *j = joints_[local_index];
    const std::uint32_t period = (period_ns_ != 0u) ? period_ns_ : 1000000u;   /* 默认 1 ms */

    /* ---- ① 使能（非阻塞请求 + 自己跑周期；SDK 的使能序列含"安全首帧"）-
       ⚠ 这里已经确认过关节是 MIT 模式（上面的守卫），所以模式是明确的。 ---- */
    jsdk_joint_request_enable(j, JSDK_MODE_MIT);
    {
        const std::uint64_t t0 = now_ns();
        while (now_ns() - t0 < 300000000ull) {   /* ≤300 ms */
            (void)jsdk_context_cycle_begin(ctx_, now_ns());
            (void)jsdk_context_cycle_end(ctx_);
            if (jsdk_joint_is_enabled(j) != 0) break;
            std::this_thread::sleep_for(std::chrono::microseconds(period / 1000u));
        }
    }
    if (jsdk_joint_is_enabled(j) == 0) {
        if (res != nullptr) {
            res->set(Status::kTimeout, Advice::kRetryFaultReset,
                     "joint '%s': enable sequence did not complete, jog aborted before any motion "
                     "(mode_state=%d, %s)",
                     bus_.joints[local_index].name,
                     static_cast<int>(jsdk_joint_get_mode_state(j)), jsdk_context_last_error(ctx_));
        }
        return Status::kTimeout;
    }

    /* ---- ② 保持 duration_ms ---- */
    const std::uint64_t t_start = now_ns();
    const std::uint64_t span = static_cast<std::uint64_t>(duration_ms) * 1000000ull;
    std::uint64_t ticks = 0u;
    while (now_ns() - t_start < span) {
        if (target.si_gain) {
            jsdk_joint_set_mit_stiffness(j, target.position, target.velocity, target.stiffness,
                                         target.damping, target.torque);
        } else {
            jsdk_joint_set_mit(j, target.position, target.velocity, target.kp, target.kd,
                               target.torque);
        }
        (void)jsdk_context_cycle_begin(ctx_, now_ns());
        (void)jsdk_context_cycle_end(ctx_);
        ++ticks;
        std::this_thread::sleep_for(std::chrono::microseconds(period / 1000u));
    }

    /* ---- ③ 安全收尾：先发安全目标 → 等 2 周期 → 失能 → 等 IDLE ---- */
    jsdk_joint_hold_position(j);
    for (unsigned k = 0u; k < 3u; ++k) {
        (void)jsdk_context_cycle_begin(ctx_, now_ns());
        (void)jsdk_context_cycle_end(ctx_);
    }
    jsdk_joint_request_disable(j);
    {
        const std::uint64_t t0 = now_ns();
        while (now_ns() - t0 < 300000000ull) {
            (void)jsdk_context_cycle_begin(ctx_, now_ns());
            (void)jsdk_context_cycle_end(ctx_);
            if (jsdk_joint_is_enabled(j) == 0) break;
            std::this_thread::sleep_for(std::chrono::microseconds(period / 1000u));
        }
    }
    const bool still_enabled = (jsdk_joint_is_enabled(j) != 0);

    if (res != nullptr) {
        res->set(still_enabled ? Status::kUnverified : Status::kOk,
                 still_enabled ? Advice::kRetryFaultReset : Advice::kNone,
                 "jog finished on '%s': %u ms / %llu ticks. Joint is %s.",
                 bus_.joints[local_index].name, duration_ms, static_cast<unsigned long long>(ticks),
                 still_enabled ? "STILL ENABLED — the disable sequence did not land; check faults"
                               : "disabled again (safe state)");
    }
    return still_enabled ? Status::kUnverified : Status::kOk;
}

}  // namespace rt
}  // namespace jr
