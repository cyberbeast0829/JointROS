/**
 * @file    jr_abi_check.cpp
 * @brief   SDK ABI 自检实现
 */

#include "jr_ros2/jr_abi_check.hpp"

#include <cstdio>
#include <cstring>

#include "joint_sdk/joint_sdk.h"

namespace jr {
namespace {

/* 期望表：名字必须与 SDK 的 k_abi_types[] 完全一致（顺序无关，按名字匹配）。 */
const AbiType k_expected[] = {
    {"jsdk_can_frame_t", static_cast<std::uint32_t>(sizeof(jsdk_can_frame_t)),
     static_cast<std::uint32_t>(alignof(jsdk_can_frame_t))},
    {"jsdk_can_hal_t", static_cast<std::uint32_t>(sizeof(jsdk_can_hal_t)),
     static_cast<std::uint32_t>(alignof(jsdk_can_hal_t))},
    {"jsdk_context_config_t", static_cast<std::uint32_t>(sizeof(jsdk_context_config_t)),
     static_cast<std::uint32_t>(alignof(jsdk_context_config_t))},
    {"jsdk_desc_config_t", static_cast<std::uint32_t>(sizeof(jsdk_desc_config_t)),
     static_cast<std::uint32_t>(alignof(jsdk_desc_config_t))},
    {"jsdk_joint_config_t", static_cast<std::uint32_t>(sizeof(jsdk_joint_config_t)),
     static_cast<std::uint32_t>(alignof(jsdk_joint_config_t))},
    {"jsdk_joint_config_snapshot_t",
     static_cast<std::uint32_t>(sizeof(jsdk_joint_config_snapshot_t)),
     static_cast<std::uint32_t>(alignof(jsdk_joint_config_snapshot_t))},
    {"jsdk_joint_feedback_t", static_cast<std::uint32_t>(sizeof(jsdk_joint_feedback_t)),
     static_cast<std::uint32_t>(alignof(jsdk_joint_feedback_t))},
    {"jsdk_bus_state_t", static_cast<std::uint32_t>(sizeof(jsdk_bus_state_t)),
     static_cast<std::uint32_t>(alignof(jsdk_bus_state_t))},
    {"jsdk_device_info_t", static_cast<std::uint32_t>(sizeof(jsdk_device_info_t)),
     static_cast<std::uint32_t>(alignof(jsdk_device_info_t))},
    {"jsdk_fault_info_t", static_cast<std::uint32_t>(sizeof(jsdk_fault_info_t)),
     static_cast<std::uint32_t>(alignof(jsdk_fault_info_t))},
    {"jsdk_value_t", static_cast<std::uint32_t>(sizeof(jsdk_value_t)),
     static_cast<std::uint32_t>(alignof(jsdk_value_t))},
    {"jsdk_unit_scale_t", static_cast<std::uint32_t>(sizeof(jsdk_unit_scale_t)),
     static_cast<std::uint32_t>(alignof(jsdk_unit_scale_t))},
    {"jsdk_param_req_t", static_cast<std::uint32_t>(sizeof(jsdk_param_req_t)),
     static_cast<std::uint32_t>(alignof(jsdk_param_req_t))},
    {"jsdk_group_target_t", static_cast<std::uint32_t>(sizeof(jsdk_group_target_t)),
     static_cast<std::uint32_t>(alignof(jsdk_group_target_t))},
    {"jsdk_desc_info_t", static_cast<std::uint32_t>(sizeof(jsdk_desc_info_t)),
     static_cast<std::uint32_t>(alignof(jsdk_desc_info_t))},
};

const jsdk_abi_type_t *find_lib_type(const jsdk_abi_type_t *table, std::size_t count,
                                     const char *name) noexcept
{
    for (std::size_t i = 0u; i < count; ++i) {
        if (std::strcmp(table[i].name, name) == 0) return &table[i];
    }
    return nullptr;
}

}  // namespace

const AbiType *expected_abi_types() noexcept { return k_expected; }

unsigned expected_abi_type_count() noexcept
{
    return static_cast<unsigned>(sizeof(k_expected) / sizeof(k_expected[0]));
}

Status compare_abi_types(const AbiType *expected, unsigned count, AbiReport *out) noexcept
{
    if (out != nullptr) {
        out->ok = false;
        out->checked = 0u;
        out->mismatches = 0u;
        out->detail[0] = '\0';
    }

    std::size_t lib_count = 0u;
    const jsdk_abi_type_t *lib = jsdk_abi_types(&lib_count);
    if (lib == nullptr) {
        if (out != nullptr) std::snprintf(out->detail, sizeof(out->detail), "jsdk_abi_types() returned NULL");
        return Status::kInternal;
    }

    const AbiType *exp = (expected != nullptr) ? expected : expected_abi_types();
    const unsigned n = (expected != nullptr) ? count : expected_abi_type_count();

    for (unsigned i = 0u; i < n; ++i) {
        const jsdk_abi_type_t *found = find_lib_type(lib, lib_count, exp[i].name);
        if (out != nullptr) ++out->checked;
        if (found == nullptr) {
            if (out != nullptr) {
                ++out->mismatches;
                if (out->detail[0] == '\0') {
                    std::snprintf(out->detail, sizeof(out->detail),
                                  "type '%s' missing from the linked library's ABI table "
                                  "(header/library version mismatch)",
                                  exp[i].name);
                }
            }
            return Status::kInternal;
        }
        if (found->size != exp[i].size || found->align != exp[i].align) {
            if (out != nullptr) {
                ++out->mismatches;
                if (out->detail[0] == '\0') {
                    std::snprintf(out->detail, sizeof(out->detail),
                                  "type '%s': header says size=%u align=%u, linked library says "
                                  "size=%u align=%u -> DO NOT RUN with this library",
                                  exp[i].name, exp[i].size, exp[i].align, found->size, found->align);
                }
            }
            return Status::kInternal;
        }
    }

    /* 反向：库里有的类型我们也应该认识（否则说明库比头文件新，同样危险）。 */
    if (lib_count != static_cast<std::size_t>(n)) {
        if (out != nullptr) {
            std::snprintf(out->detail, sizeof(out->detail),
                          "type-table length mismatch: header expects %u entries, linked library "
                          "reports %u",
                          n, static_cast<unsigned>(lib_count));
        }
        return Status::kInternal;
    }

    if (out != nullptr) out->ok = true;
    return Status::kOk;
}

Status check_sdk_abi(AbiReport *out) noexcept
{
    if (out != nullptr) {
        out->ok = false;
        out->checked = 0u;
        out->mismatches = 0u;
        out->detail[0] = '\0';
    }

    /* ① 后端身份：绝不能把 EtherCAT 版后端链进来（SDK 明确：同符号、互斥链接）。 */
    const char *backend = jsdk_backend_name();
    if (backend == nullptr || std::strcmp(backend, "cyberbeast-can") != 0) {
        if (out != nullptr) {
            std::snprintf(out->detail, sizeof(out->detail),
                          "wrong SDK backend: expected 'cyberbeast-can', got '%s' "
                          "(the IgH/SOEM EtherCAT backends share the jsdk_* symbols and MUST NOT "
                          "be linked together)",
                          backend != nullptr ? backend : "(null)");
        }
        return Status::kInternal;
    }

    /* ② ABI 版本。 */
    const std::uint32_t abi = jsdk_abi_version();
    if (abi != JSDK_ABI_VERSION_CAN) {
        if (out != nullptr) {
            std::snprintf(out->detail, sizeof(out->detail),
                          "SDK ABI version mismatch: header 0x%08X, library 0x%08X",
                          static_cast<unsigned>(JSDK_ABI_VERSION_CAN),
                          static_cast<unsigned>(abi));
        }
        return Status::kInternal;
    }

    /* ③ 逐个结构体尺寸/对齐对拍。 */
    const Status st = compare_abi_types(nullptr, 0u, out);
    if (st != Status::kOk) return st;

    /* ④ 上下文尺寸合理性：必须能装下"本包配置的最大关节数"。 */
    const std::size_t ctx_size = jsdk_context_size(nullptr);
    if (ctx_size == 0u) {
        if (out != nullptr) std::snprintf(out->detail, sizeof(out->detail), "jsdk_context_size() == 0");
        return Status::kInternal;
    }
    if (ctx_size > JSDK_CONTEXT_MAX_SIZE) {
        if (out != nullptr) {
            std::snprintf(out->detail, sizeof(out->detail),
                          "jsdk_context_size()=%u exceeds JSDK_CONTEXT_MAX_SIZE=%u",
                          static_cast<unsigned>(ctx_size),
                          static_cast<unsigned>(JSDK_CONTEXT_MAX_SIZE));
        }
        return Status::kInternal;
    }

    if (out != nullptr) {
        out->ok = true;
        std::snprintf(out->detail, sizeof(out->detail),
                      "backend=%s abi=0x%08X types=%u ctx_size=%u max_joints=%u", backend,
                      static_cast<unsigned>(abi), out->checked,
                      static_cast<unsigned>(ctx_size),
                      static_cast<unsigned>(JSDK_MAX_JOINTS_STATIC));
    }
    return Status::kOk;
}

}  // namespace jr
