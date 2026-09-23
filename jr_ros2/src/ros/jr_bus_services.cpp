/**
 * @file    jr_bus_services.cpp
 * @brief   `jr_bus` 的服务层（DESIGN §6.2 的 19 个服务）
 *
 * @par 三条硬规则（都在这里落地，不要绕过）
 *  1. **ADR-7 安全暂停窗口**：参数/描述符/标定/使能这些操作会阻塞，且 SDK 明确禁止在
 *     关节使能时做（参数帧会挤掉控制帧 → 触发设备看门狗）。所以每次操作都是
 *     `pause()`（先安全失能再交出所有权）→ 操作 → 按需恢复 → `resume()`。
 *  2. **§8.5 写闸门**：参数写要 `params.allow_write` + `confirm`；Flash 持久化要
 *     `params.allow_flash_persist` + `confirm`；改通讯配置/复位要 `confirm`。
 *     闸门不通过就**明确拒绝并说清缺什么**，不做"默默降级"。
 *  3. **逐项结果**：批量服务一律返回逐关节结果（不允许"部分成功"含糊过去）；
 *     `verified` 与 `requested` 分开（ADR-8：说得不能比知道的多）。
 */

#include "jr_ros2/ros/jr_bus_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <rmw/qos_profiles.h>

#include <jr_interfaces/msg/endpoint_info.hpp>
#include <jr_interfaces/msg/joint_result.hpp>
#include <jr_interfaces/msg/param_value.hpp>
#include <jr_interfaces/msg/param_write_result.hpp>
#include <jr_interfaces/srv/calibrate.hpp>
#include <jr_interfaces/srv/export_descriptor.hpp>
#include <jr_interfaces/srv/fault_reset.hpp>
#include <jr_interfaces/srv/get_bus_stats.hpp>
#include <jr_interfaces/srv/get_descriptor_info.hpp>
#include <jr_interfaces/srv/get_device_info.hpp>
#include <jr_interfaces/srv/home.hpp>
#include <jr_interfaces/srv/import_descriptor.hpp>
#include <jr_interfaces/srv/jog.hpp>
#include <jr_interfaces/srv/list_endpoints.hpp>
#include <jr_interfaces/srv/lookup_endpoint.hpp>
#include <jr_interfaces/srv/publish_heartbeat_hint.hpp>
#include <jr_interfaces/srv/read_params.hpp>
#include <jr_interfaces/srv/reset_device.hpp>
#include <jr_interfaces/srv/save_config.hpp>
#include <jr_interfaces/srv/set_enabled.hpp>
#include <jr_interfaces/srv/set_node_id.hpp>
#include <jr_interfaces/srv/set_zero.hpp>
#include <jr_interfaces/srv/write_params.hpp>

namespace jr {
namespace ros {

namespace {

/** 服务层的通用上限（内存里枚举端点，不落盘）。 */
constexpr unsigned kEndpointCap = 4096u;
/** 点动时长硬上限（DESIGN §6.2：超了直接拒绝，不做"帮你截断"）。 */
constexpr double kJogMaxDurationS = 10.0;

/**
 * 服务 QoS 的**发行版兼容层**。
 *
 * ⚠ `Node::create_service` 的第 3 个参数换过类型：
 *   - Humble 及更早：`const rmw_qos_profile_t &`（默认 `rmw_qos_profile_services_default`）
 *   - Iron 之后（Jazzy/Lyrical）：`const rclcpp::QoS &`（默认 `rclcpp::ServicesQoS()`）
 *   两边默认值**语义相同**（`ServicesQoS()` 就是按 `rmw_qos_profile_services_default` 构造的）。
 *
 * 为什么用 `if constexpr` 探测而不是 `#if` 判发行版：探测的是**当前 rclcpp 的真实可调用性**，
 * 换发行版时由编译器直接告诉我们；发行版宏则要靠人记得改（本仓库已经栽过"CMake 探测静默失败"）。
 * 也不能"同时提供两种隐式转换"地传一个实参 —— Jazzy 上两种重载都在，那样**歧义**。
 */
namespace srv_qos_compat {

/** 新版 rclcpp（Iron+）：第 3 个参数是 `rclcpp::QoS`。 */
template <class SrvT, class = void>
struct takes_qos_object : std::false_type {};

template <class SrvT>
struct takes_qos_object<
    SrvT, std::void_t<decltype(std::declval<rclcpp_lifecycle::LifecycleNode &>()
                                   .template create_service<SrvT>(
                                       std::declval<const std::string &>(),
                                       std::declval<std::function<void(
                                           typename SrvT::Request::SharedPtr,
                                           typename SrvT::Response::SharedPtr)>>(),
                                       std::declval<const rclcpp::QoS &>(),
                                       std::declval<rclcpp::CallbackGroup::SharedPtr>()))>>
    : std::true_type {};

/** 旧版 rclcpp（Humble）：第 3 个参数是 `rmw_qos_profile_t`。
 *
 *  ⚠⚠ **只有当新版不存在时才去探测它**。Jazzy 上旧重载仍然"存在"，只是被标了
 *  `[[deprecated]]` —— 而 GCC 在**未求值上下文**（`decltype`）里也会发
 *  `-Wdeprecated-declarations`，于是一行探测就把"零告警"打破。
 *  非类型参数 `HasQos` 就是用来"短路"这个探测的（新版在 → 根本不实例化旧探测）。 */
template <class SrvT, bool HasQos, class = void>
struct takes_rmw_profile_impl : std::false_type {};

template <class SrvT>
struct takes_rmw_profile_impl<
    SrvT, false, std::void_t<decltype(std::declval<rclcpp_lifecycle::LifecycleNode &>()
                                          .template create_service<SrvT>(
                                              std::declval<const std::string &>(),
                                              std::declval<std::function<void(
                                                  typename SrvT::Request::SharedPtr,
                                                  typename SrvT::Response::SharedPtr)>>(),
                                              rmw_qos_profile_services_default,
                                              std::declval<rclcpp::CallbackGroup::SharedPtr>()))>>
    : std::true_type {};

template <class SrvT>
struct takes_rmw_profile : takes_rmw_profile_impl<SrvT, takes_qos_object<SrvT>::value> {};

}  // namespace srv_qos_compat

/** 编译期守卫：两种签名**至少有一种**认 —— 都不认说明 rclcpp 又改成第三种样子了：
 *  这里编译期就红，而不是等到现场“服务调用莫名超时”才发现 QoS 没生效。 */
static_assert(srv_qos_compat::takes_qos_object<jr_interfaces::srv::GetBusStats>::value ||
                  srv_qos_compat::takes_rmw_profile<jr_interfaces::srv::GetBusStats>::value,
              "create_service 的 QoS 参数既不认 rclcpp::QoS 也不认 rmw_qos_profile_t（rclcpp 改签名了？）");

/** 建一个服务（服务默认 QoS），把上面的签名差异收在**这一处**。
 *
 *  ⚠ 顺序要紧：**优先用新版（`rclcpp::QoS`）**。旧签名分支只在"新版不存在"时才实例化
 *  （Humble）—— 这样既不会在 Jazzy 上吃 `-Wdeprecated-declarations`，
 *  也不用 `#if` 猜发行版。 */
template <class SrvT, class NodeT, class CbT>
typename rclcpp::Service<SrvT>::SharedPtr make_service(NodeT &node, const char *name, CbT cb,
                                                      const rclcpp::CallbackGroup::SharedPtr &group)
{
    if constexpr (srv_qos_compat::takes_qos_object<SrvT>::value) {
        return node.template create_service<SrvT>(name, cb, rclcpp::ServicesQoS(), group);
    } else {
        static_assert(srv_qos_compat::takes_rmw_profile<SrvT>::value,
                      "两种 create_service 签名都不认（rclcpp 改签名了？）");
        return node.template create_service<SrvT>(name, cb, rmw_qos_profile_services_default, group);
    }
}

std::uint8_t st_code(Status s) noexcept
{
    return static_cast<std::uint8_t>(static_cast<std::int32_t>(s));
}

std::uint8_t type_code(ParamType t) noexcept { return static_cast<std::uint8_t>(t); }

/** 把结果文本刷进日志（服务出问题时，日志里必须能看到同一句话）。 */
void log_result(rclcpp_lifecycle::LifecycleNode *n, const char *what, const Result &r)
{
    if (r.ok()) {
        RCLCPP_INFO(n->get_logger(), "%s: %s", what, r.message);
    } else {
        RCLCPP_WARN(n->get_logger(), "%s failed (%s, advice=%s): %s", what, jr::to_string(r.status),
                    jr::to_string(r.advice), r.message);
    }
}

}  // namespace

/* ==========================================================================
 * 服务层实现
 * ======================================================================== */

struct JrBusServices {
    explicit JrBusServices(JrBusNode *n) : node(n) {}

    JrBusNode *node;

    rclcpp::Service<jr_interfaces::srv::SetEnabled>::SharedPtr set_enabled;
    rclcpp::Service<jr_interfaces::srv::Calibrate>::SharedPtr calibrate;
    rclcpp::Service<jr_interfaces::srv::Home>::SharedPtr home;
    rclcpp::Service<jr_interfaces::srv::SetZero>::SharedPtr set_zero;
    rclcpp::Service<jr_interfaces::srv::SaveConfig>::SharedPtr save_config;
    rclcpp::Service<jr_interfaces::srv::ResetDevice>::SharedPtr reset_device;
    rclcpp::Service<jr_interfaces::srv::SetNodeId>::SharedPtr set_node_id;
    rclcpp::Service<jr_interfaces::srv::FaultReset>::SharedPtr fault_reset;
    rclcpp::Service<jr_interfaces::srv::Jog>::SharedPtr jog_srv;
    rclcpp::Service<jr_interfaces::srv::ReadParams>::SharedPtr read_params;
    rclcpp::Service<jr_interfaces::srv::WriteParams>::SharedPtr write_params;
    rclcpp::Service<jr_interfaces::srv::ListEndpoints>::SharedPtr list_endpoints;
    rclcpp::Service<jr_interfaces::srv::LookupEndpoint>::SharedPtr lookup_endpoint;
    rclcpp::Service<jr_interfaces::srv::GetDeviceInfo>::SharedPtr get_device_info;
    rclcpp::Service<jr_interfaces::srv::GetBusStats>::SharedPtr get_bus_stats;
    rclcpp::Service<jr_interfaces::srv::GetDescriptorInfo>::SharedPtr get_descriptor_info;
    rclcpp::Service<jr_interfaces::srv::ExportDescriptor>::SharedPtr export_descriptor;
    rclcpp::Service<jr_interfaces::srv::ImportDescriptor>::SharedPtr import_descriptor;
    rclcpp::Service<jr_interfaces::srv::PublishHeartbeatHint>::SharedPtr heartbeat_hint;

    /* ---------------- 生命周期 ---------------- */

    void create();
    void destroy();

    /** 把成员函数绑成**具体类型**的可调用体（服务回调）：
     *  ⚠ 不能用 `[this](auto req, auto resp){...}` 这种**泛型 lambda** —— `create_service`
     *  的回调是模板参数，泛型 lambda 推不出来（编译期直接 "no matching function"）。 */
    template <class SrvT>
    auto bind_srv(void (JrBusServices::*fn)(typename SrvT::Request::SharedPtr,
                                            typename SrvT::Response::SharedPtr))
    {
        return [this, fn](typename SrvT::Request::SharedPtr req,
                          typename SrvT::Response::SharedPtr resp) { (this->*fn)(req, resp); };
    }

    /* ======================================================================
     * 公共工具
     * ==================================================================== */

    rt::TickGroup *tg() const noexcept { return node->tg_.get(); }
    rt::BusRuntime &bus() const noexcept { return tg()->bus(node->bus_index_); }
    const Config &cfg() const noexcept { return node->cfg_; }
    const BusCfg &bus_cfg() const noexcept { return node->cfg_.buses[node->bus_index_]; }

    /** 请求里的 `bus` 字段：空 = 本节点；写了但不匹配 → 明确拒绝（客户常把多总线搞混）。 */
    bool check_bus(const std::string &req_bus, std::string *err) const
    {
        if (tg() == nullptr) {
            *err = "the node is not active (call the lifecycle activate transition first)";
            return false;
        }
        if (req_bus.empty() || req_bus == bus_cfg().name) return true;
        *err = "this service instance owns bus '" + std::string(bus_cfg().name) + "', not '" + req_bus +
               "': multi-bus setups run one node per bus — call the service on the right namespace";
        return false;
    }

    /** 关节名列表 → 总线内序号（空 = 全部）。非法名字**不猜**，直接拒绝并列出已知关节。 */
    bool select(const std::vector<std::string> &names, std::vector<unsigned> *out,
                std::string *err) const
    {
        out->clear();
        if (names.empty()) {
            for (unsigned j = 0u; j < bus_cfg().joint_count; ++j) out->push_back(j);
            return true;
        }
        for (const std::string &n : names) {
            const int idx = node->local_index_of(n);
            if (idx < 0) {
                std::string known;
                for (unsigned j = 0u; j < bus_cfg().joint_count; ++j) {
                    if (!known.empty()) known += ", ";
                    known += bus_cfg().joints[j].name;
                }
                *err = "joint '" + n + "' is not on bus '" + std::string(bus_cfg().name) +
                       "' (known: " + known + ")";
                return false;
            }
            out->push_back(static_cast<unsigned>(idx));
        }
        return true;
    }

    void fill_result(jr_interfaces::msg::JointResult *r, const char *name, const Status st,
                     const char *msg) const
    {
        r->name = name;
        r->success = (st == Status::kOk);
        r->status = st_code(st);
        r->message = msg;
    }

    /** 逐个关节填结果（同一句话 + 同一状态码；细节在 `message` 里）。 */
    void fill_results_same(std::vector<jr_interfaces::msg::JointResult> *out,
                           const std::vector<unsigned> &sel, const Status st, const char *msg) const
    {
        out->clear();
        for (unsigned j : sel) {
            jr_interfaces::msg::JointResult r;
            fill_result(&r, bus_cfg().joints[j].name, st, msg);
            out->push_back(std::move(r));
        }
    }

    bool all_ok(const std::vector<jr_interfaces::msg::JointResult> &rs) const
    {
        for (const auto &r : rs) {
            if (!r.success) return false;
        }
        return true;
    }

    /* ---------------- ADR-7：安全暂停窗口 ---------------- */

    /**
     * 进入窗口：记录"哪些关节现在使能"，然后 `pause()`（让 tick 线程先安全失能并交出
     * context 所有权）。**必须**与 `exit_window()` 配对（异常/早退也要走到）。
     */
    bool enter_window(std::vector<unsigned> *was_enabled, std::string *err)
    {
        was_enabled->clear();
        rt::TickGroup *g = tg();
        if (g == nullptr) {
            *err = "the node is not active";
            return false;
        }
        if (g->bus_count() == 0u) {
            *err = "the tick group has no buses";
            return false;
        }
        for (unsigned j = 0u; j < bus_cfg().joint_count; ++j) {
            if (g->bus(node->bus_index_).joint_enabled(j)) was_enabled->push_back(j);
        }
        Result r;
        if (g->pause(&r, 5000u) != Status::kOk) {
            *err = std::string("cannot enter the safe pause window: ") + r.message;
            return false;
        }
        return true;
    }

    /**
     * 退出窗口：按需把"操作前使能的关节"恢复，然后把所有权还给 tick 线程。
     * `skip` = 本次请求**专门要改动**的关节（它们不该被"恢复"覆盖）。
     */
    void exit_window(const std::vector<unsigned> &was_enabled, const std::vector<unsigned> &skip,
                     bool resume)
    {
        rt::TickGroup *g = tg();
        if (g == nullptr) return;
        if (resume) {
            std::vector<unsigned> restore;
            for (unsigned j : was_enabled) {
                if (std::find(skip.begin(), skip.end(), j) == skip.end()) restore.push_back(j);
            }
            if (!restore.empty()) {
                Result r;
                const Status st = g->bus(node->bus_index_)
                                      .set_enabled(restore.data(), static_cast<unsigned>(restore.size()),
                                                   true, cfg().safety.require_calibrated, &r);
                if (st != Status::kOk) {
                    RCLCPP_WARN(node->get_logger(),
                                "resume=true but re-enabling the previously enabled joints failed: %s "
                                "(they stay disabled — read back `enabled` before acting)",
                                r.message);
                }
            }
        }
        g->resume();
    }

    /** 窗口 + 恢复的 RAII（早退也不用记得手动配平）。 */
    struct Guard {
        JrBusServices *s;
        std::vector<unsigned> was_enabled;
        std::vector<unsigned> skip;
        bool resume = true;
        bool open = false;

        ~Guard()
        {
            if (open) s->exit_window(was_enabled, skip, resume);
        }
    };

    bool open_window(Guard *guard, const std::vector<unsigned> &skip, bool resume, std::string *err)
    {
        guard->s = this;
        guard->skip = skip;
        guard->resume = resume;
        if (!enter_window(&guard->was_enabled, err)) return false;
        guard->open = true;
        return true;
    }

    /** 闸门失败时统一的回应（把"缺哪个开关"写清楚）。 */

    /* ======================================================================
     * 运动 / 生命周期类
     * ==================================================================== */

    void on_set_enabled(const jr_interfaces::srv::SetEnabled::Request::SharedPtr req,
                        jr_interfaces::srv::SetEnabled::Response::SharedPtr resp);
    void on_calibrate(const jr_interfaces::srv::Calibrate::Request::SharedPtr req,
                      jr_interfaces::srv::Calibrate::Response::SharedPtr resp);
    void on_home(const jr_interfaces::srv::Home::Request::SharedPtr req,
                 jr_interfaces::srv::Home::Response::SharedPtr resp);
    void on_set_zero(const jr_interfaces::srv::SetZero::Request::SharedPtr req,
                     jr_interfaces::srv::SetZero::Response::SharedPtr resp);
    void on_save_config(const jr_interfaces::srv::SaveConfig::Request::SharedPtr req,
                        jr_interfaces::srv::SaveConfig::Response::SharedPtr resp);
    void on_reset_device(const jr_interfaces::srv::ResetDevice::Request::SharedPtr req,
                         jr_interfaces::srv::ResetDevice::Response::SharedPtr resp);
    void on_set_node_id(const jr_interfaces::srv::SetNodeId::Request::SharedPtr req,
                        jr_interfaces::srv::SetNodeId::Response::SharedPtr resp);
    void on_fault_reset(const jr_interfaces::srv::FaultReset::Request::SharedPtr req,
                        jr_interfaces::srv::FaultReset::Response::SharedPtr resp);
    void on_jog(const jr_interfaces::srv::Jog::Request::SharedPtr req,
                jr_interfaces::srv::Jog::Response::SharedPtr resp);

    /* ======================================================================
     * 参数 / 描述符 / 诊断 / 产线类
     * ==================================================================== */

    void on_read_params(const jr_interfaces::srv::ReadParams::Request::SharedPtr req,
                        jr_interfaces::srv::ReadParams::Response::SharedPtr resp);
    void on_write_params(const jr_interfaces::srv::WriteParams::Request::SharedPtr req,
                         jr_interfaces::srv::WriteParams::Response::SharedPtr resp);
    void on_list_endpoints(const jr_interfaces::srv::ListEndpoints::Request::SharedPtr req,
                           jr_interfaces::srv::ListEndpoints::Response::SharedPtr resp);
    void on_lookup_endpoint(const jr_interfaces::srv::LookupEndpoint::Request::SharedPtr req,
                            jr_interfaces::srv::LookupEndpoint::Response::SharedPtr resp);
    void on_get_device_info(const jr_interfaces::srv::GetDeviceInfo::Request::SharedPtr req,
                            jr_interfaces::srv::GetDeviceInfo::Response::SharedPtr resp);
    void on_get_bus_stats(const jr_interfaces::srv::GetBusStats::Request::SharedPtr req,
                          jr_interfaces::srv::GetBusStats::Response::SharedPtr resp);
    void on_get_descriptor_info(const jr_interfaces::srv::GetDescriptorInfo::Request::SharedPtr req,
                                jr_interfaces::srv::GetDescriptorInfo::Response::SharedPtr resp);
    void on_export_descriptor(const jr_interfaces::srv::ExportDescriptor::Request::SharedPtr req,
                              jr_interfaces::srv::ExportDescriptor::Response::SharedPtr resp);
    void on_import_descriptor(const jr_interfaces::srv::ImportDescriptor::Request::SharedPtr req,
                              jr_interfaces::srv::ImportDescriptor::Response::SharedPtr resp);
    void on_heartbeat_hint(const jr_interfaces::srv::PublishHeartbeatHint::Request::SharedPtr req,
                           jr_interfaces::srv::PublishHeartbeatHint::Response::SharedPtr resp);

    /* ======================================================================
     * 复位之后必须重新 configure（设备重新上电自检，描述符/量程都得重来）
     * ==================================================================== */

    void reconfigure_after_reset(std::string *note)
    {
        Result r;
        const Status st = tg()->configure_buses(&r);
        if (st != Status::kOk) {
            *note = std::string("bus re-configure after the device reset FAILED: ") + r.message;
            return;
        }
        *note = "bus re-configured after the device reset (descriptor + ranges re-read)";
    }
};

/* ==========================================================================
 * 创建 / 销毁
 * ======================================================================== */

void JrBusServices::create()
{
    /* 阻塞型服务放在**独立回调组**里（§5.1 的 T3）：不能让一次标定把状态发布与命令接收卡住。

       ⚠⚠ 回调组必须由**我们**持有强引用（存在节点上）：rclcpp 的 `NodeBase::callback_groups_`
       与 Subscription/Service 内部存的都是 **weak_ptr** —— 把组建成本函数的局部变量，
       `create()` 一返回组就析构，结果是：服务在 `ros2 service list` 里看得见、
       `wait_for_service()` 也返回成功，但请求**永远不会被 executor 派发**（回调不执行、
       客户端只看到超时）。实测踩过：默认组一切正常，换成本地自定义组就全超时。 */
    if (node->svc_cbg_ == nullptr) {
        node->svc_cbg_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    }
    auto cbg = node->svc_cbg_;
    namespace S = jr_interfaces::srv;   /* 命名空间别名（`using S = ...` 对命名空间不成立） */

    set_enabled = make_service<S::SetEnabled>(*node,
        "~/set_enabled", bind_srv<S::SetEnabled>(&JrBusServices::on_set_enabled), cbg);
    calibrate = make_service<S::Calibrate>(*node,
        "~/calibrate", bind_srv<S::Calibrate>(&JrBusServices::on_calibrate), cbg);
    home = make_service<S::Home>(*node, "~/home", bind_srv<S::Home>(&JrBusServices::on_home), cbg);
    set_zero = make_service<S::SetZero>(*node, "~/set_zero",
                                               bind_srv<S::SetZero>(&JrBusServices::on_set_zero), cbg);
    save_config = make_service<S::SaveConfig>(*node,
        "~/save_config", bind_srv<S::SaveConfig>(&JrBusServices::on_save_config), cbg);
    reset_device = make_service<S::ResetDevice>(*node,
        "~/reset_device", bind_srv<S::ResetDevice>(&JrBusServices::on_reset_device), cbg);
    set_node_id = make_service<S::SetNodeId>(*node,
        "~/set_node_id", bind_srv<S::SetNodeId>(&JrBusServices::on_set_node_id), cbg);
    fault_reset = make_service<S::FaultReset>(*node,
        "~/fault_reset", bind_srv<S::FaultReset>(&JrBusServices::on_fault_reset), cbg);
    jog_srv = make_service<S::Jog>(*node, "~/jog", bind_srv<S::Jog>(&JrBusServices::on_jog), cbg);
    read_params = make_service<S::ReadParams>(*node,
        "~/read_params", bind_srv<S::ReadParams>(&JrBusServices::on_read_params), cbg);
    write_params = make_service<S::WriteParams>(*node,
        "~/write_params", bind_srv<S::WriteParams>(&JrBusServices::on_write_params), cbg);
    list_endpoints = make_service<S::ListEndpoints>(*node,
        "~/list_endpoints", bind_srv<S::ListEndpoints>(&JrBusServices::on_list_endpoints), cbg);
    lookup_endpoint = make_service<S::LookupEndpoint>(*node,
        "~/lookup_endpoint", bind_srv<S::LookupEndpoint>(&JrBusServices::on_lookup_endpoint), cbg);
    get_device_info = make_service<S::GetDeviceInfo>(*node,
        "~/get_device_info", bind_srv<S::GetDeviceInfo>(&JrBusServices::on_get_device_info), cbg);
    get_bus_stats = make_service<S::GetBusStats>(*node,
        "~/get_bus_stats", bind_srv<S::GetBusStats>(&JrBusServices::on_get_bus_stats), cbg);
    get_descriptor_info = make_service<S::GetDescriptorInfo>(*node,
        "~/get_descriptor_info", bind_srv<S::GetDescriptorInfo>(&JrBusServices::on_get_descriptor_info), cbg);
    export_descriptor = make_service<S::ExportDescriptor>(*node,
        "~/export_descriptor", bind_srv<S::ExportDescriptor>(&JrBusServices::on_export_descriptor), cbg);
    import_descriptor = make_service<S::ImportDescriptor>(*node,
        "~/import_descriptor", bind_srv<S::ImportDescriptor>(&JrBusServices::on_import_descriptor), cbg);
    heartbeat_hint = make_service<S::PublishHeartbeatHint>(*node,
        "~/publish_heartbeat_hint",
        bind_srv<S::PublishHeartbeatHint>(&JrBusServices::on_heartbeat_hint), cbg);

    RCLCPP_INFO(node->get_logger(), "services up (19): motion/lifecycle + params/descriptor/diagnostics");
}

void JrBusServices::destroy()
{
    set_enabled.reset();
    calibrate.reset();
    home.reset();
    set_zero.reset();
    save_config.reset();
    reset_device.reset();
    set_node_id.reset();
    fault_reset.reset();
    jog_srv.reset();
    read_params.reset();
    write_params.reset();
    list_endpoints.reset();
    lookup_endpoint.reset();
    get_device_info.reset();
    get_bus_stats.reset();
    get_descriptor_info.reset();
    export_descriptor.reset();
    import_descriptor.reset();
    heartbeat_hint.reset();
    /* 服务都放掉了，回调组也可以放（下次 create() 会重新建）。 */
    if (node != nullptr) node->svc_cbg_.reset();
}

/* ==========================================================================
 * 运动 / 生命周期类
 * ======================================================================== */

void JrBusServices::on_set_enabled(const jr_interfaces::srv::SetEnabled::Request::SharedPtr req,
                                  jr_interfaces::srv::SetEnabled::Response::SharedPtr resp)
{
    std::string err;
    std::vector<unsigned> sel;
    if (!select(req->joints, &sel, &err)) {
        resp->success = false;
        resp->message = err;
        return;
    }

    Guard g;
    if (!open_window(&g, sel, req->resume, &err)) {
        resp->success = false;
        resp->message = err;
        return;
    }

    Result r;
    const Status st = bus().set_enabled(sel.data(), static_cast<unsigned>(sel.size()), req->enable,
                                        cfg().safety.require_calibrated, &r);
    log_result(node, req->enable ? "SetEnabled(true)" : "SetEnabled(false)", r);

    /* 逐关节结果用**读回**的事实填（不是"我请求了什么"）。 */
    std::vector<jr_interfaces::msg::JointResult> results;
    for (unsigned j : sel) {
        jr_interfaces::msg::JointResult jr_msg;
        const bool now = bus().joint_enabled(j);
        const bool ok = (now == req->enable);
        char buf[192] = {};
        std::snprintf(buf, sizeof(buf), "enabled=%s after the operation (requested %s)", now ? "true" : "false",
                      req->enable ? "true" : "false");
        fill_result(&jr_msg, bus_cfg().joints[j].name, ok ? Status::kOk : st, buf);
        results.push_back(std::move(jr_msg));
    }

    resp->success = (st == Status::kOk) && all_ok(results);
    resp->message = r.message;
    resp->results = results;
}

void JrBusServices::on_calibrate(const jr_interfaces::srv::Calibrate::Request::SharedPtr req,
                                 jr_interfaces::srv::Calibrate::Response::SharedPtr resp)
{
    std::string err;
    std::vector<unsigned> sel;
    if (!select(req->joints, &sel, &err)) {
        resp->success = false;
        resp->message = err;
        return;
    }
    /* `timeout_ms` 是**客户端期望**；核心库用配置里的 `state_timeout_ms`（标定最坏几十秒）。
       客户端给了更小的值时说清楚（不假装听它的）。 */
    if (req->timeout_ms != 0u) {
        const std::uint32_t core_ms = bus_cfg().state_timeout_ms;
        if (core_ms != 0u && req->timeout_ms < core_ms) {
            RCLCPP_WARN(node->get_logger(),
                        "Calibrate: requested timeout_ms=%u is smaller than the configured "
                        "state_timeout_ms=%u — the core uses the configured value.",
                        req->timeout_ms, core_ms);
        }
    }

    Guard g;
    if (!open_window(&g, sel, true, &err)) {
        resp->success = false;
        resp->message = err;
        return;
    }

    Result r;
    const Status st = bus().calibrate(sel.data(), static_cast<unsigned>(sel.size()), &r);
    log_result(node, "Calibrate", r);

    std::vector<jr_interfaces::msg::JointResult> results;
    std::vector<bool> verified;
    for (unsigned j : sel) {
        jr_interfaces::msg::JointResult jr_msg;
        /* 成功判据是**读回**（核心已刷新报告）：calibrated 标志。 */
        const bool ok = bus().report().joint[j].calibrated;
        char buf[192] = {};
        std::snprintf(buf, sizeof(buf), "pre_calibrated read-back: %s", ok ? "true" : "false");
        fill_result(&jr_msg, bus_cfg().joints[j].name, ok ? Status::kOk : Status::kUnverified, buf);
        results.push_back(std::move(jr_msg));
        verified.push_back(ok);
    }
    resp->success = (st == Status::kOk) && all_ok(results);
    resp->message = r.message;
    resp->results = results;
    resp->verified = verified;
}

void JrBusServices::on_home(const jr_interfaces::srv::Home::Request::SharedPtr req,
                            jr_interfaces::srv::Home::Response::SharedPtr resp)
{
    std::string err;
    std::vector<unsigned> sel;
    if (!select(req->joints, &sel, &err)) {
        resp->success = false;
        resp->message = err;
        return;
    }
    Guard g;
    if (!open_window(&g, sel, true, &err)) {
        resp->success = false;
        resp->message = err;
        return;
    }
    Result r;
    const Status st = bus().home(sel.data(), static_cast<unsigned>(sel.size()), &r);
    log_result(node, "Home", r);

    /* `verified`：homing 不改变标定标志，"verified" 只能表示"整条序列跑完且没报错"。 */
    std::vector<bool> verified(sel.size(), st == Status::kOk);
    fill_results_same(&resp->results, sel, st, r.message);
    resp->success = (st == Status::kOk);
    resp->message = r.message;
    resp->verified = verified;
}

void JrBusServices::on_set_zero(const jr_interfaces::srv::SetZero::Request::SharedPtr req,
                                jr_interfaces::srv::SetZero::Response::SharedPtr resp)
{
    std::string err;
    std::vector<unsigned> sel;
    if (!select(req->joints, &sel, &err)) {
        resp->success = false;
        resp->message = err;
        return;
    }
    Guard g;
    if (!open_window(&g, sel, true, &err)) {
        resp->success = false;
        resp->message = err;
        return;
    }
    Result r;
    const Status st = bus().set_zero_here(sel.data(), static_cast<unsigned>(sel.size()), &r);
    log_result(node, "SetZero", r);
    fill_results_same(&resp->results, sel, st, r.message);
    resp->success = (st == Status::kOk);
    resp->message = r.message;
    resp->persisted = false;   /* 本服务**不**写 Flash（要持久化请调 SaveConfig） */
}

void JrBusServices::on_save_config(const jr_interfaces::srv::SaveConfig::Request::SharedPtr req,
                                   jr_interfaces::srv::SaveConfig::Response::SharedPtr resp)
{
    if (!req->confirm) {
        resp->success = false;
        resp->message = "SaveConfig writes the device flash: pass confirm=true (DESIGN §8.5)";
        return;
    }
    if (!cfg().safety.allow_flash_persist) {
        resp->success = false;
        resp->message =
            "flash persistence is disabled (safety.allow_flash_persist=false in the config, i.e. "
            "params.allow_flash_persist): enable it explicitly before writing device flash";
        return;
    }
    std::string err;
    std::vector<unsigned> sel;
    if (!select(req->joints, &sel, &err)) {
        resp->success = false;
        resp->message = err;
        return;
    }
    Guard g;
    if (!open_window(&g, sel, true, &err)) {
        resp->success = false;
        resp->message = err;
        return;
    }
    Result r;
    const Status st = bus().save_config(sel.data(), static_cast<unsigned>(sel.size()), &r);
    log_result(node, "SaveConfig", r);
    fill_results_same(&resp->results, sel, st, r.message);
    resp->success = (st == Status::kOk);
    resp->message = r.message;
}

void JrBusServices::on_reset_device(const jr_interfaces::srv::ResetDevice::Request::SharedPtr req,
                                    jr_interfaces::srv::ResetDevice::Response::SharedPtr resp)
{
    if (!req->confirm) {
        resp->success = false;
        resp->message = "ResetDevice reboots the device: pass confirm=true (DESIGN §8.5)";
        return;
    }
    std::string err;
    std::vector<unsigned> sel;
    if (!select(req->joints, &sel, &err)) {
        resp->success = false;
        resp->message = err;
        return;
    }
    Guard g;
    if (!open_window(&g, sel, false, &err)) {   /* 复位后关节一定是断的：不恢复 */
        resp->success = false;
        resp->message = err;
        return;
    }
    Result r;
    const Status st = bus().reset_device(sel.data(), static_cast<unsigned>(sel.size()), &r);
    log_result(node, "ResetDevice", r);

    std::string note;
    if (st == Status::kOk) reconfigure_after_reset(&note);
    fill_results_same(&resp->results, sel, st, r.message);
    resp->success = (st == Status::kOk);
    resp->message = note.empty() ? std::string(r.message) : (std::string(r.message) + " | " + note);
    resp->advice = static_cast<std::uint8_t>(Advice::kNone);
    if (!note.empty() && note.find("FAILED") != std::string::npos) {
        resp->advice = static_cast<std::uint8_t>(Advice::kNeedsDeviceReset);
    }
}

void JrBusServices::on_set_node_id(const jr_interfaces::srv::SetNodeId::Request::SharedPtr req,
                                   jr_interfaces::srv::SetNodeId::Response::SharedPtr resp)
{
    if (!req->confirm) {
        resp->success = false;
        resp->message = "changing node_id changes device addressing: pass confirm=true (DESIGN §8.5)";
        return;
    }
    if (req->persist && !cfg().safety.allow_flash_persist) {
        resp->success = false;
        resp->message =
            "persist=true requires flash persistence to be allowed (params.allow_flash_persist)";
        return;
    }
    const int idx = node->local_index_of(req->name);
    if (idx < 0) {
        resp->success = false;
        resp->message = "joint '" + req->name + "' is not on bus '" + std::string(bus_cfg().name) + "'";
        return;
    }
    std::vector<unsigned> sel{static_cast<unsigned>(idx)};
    std::string err;
    Guard g;
    if (!open_window(&g, sel, false, &err)) {   /* 改完寻址会变：不自动恢复 */
        resp->success = false;
        resp->message = err;
        return;
    }
    Result r;
    const Status st = bus().set_node_id(static_cast<unsigned>(idx), req->new_id, req->persist, &r);
    log_result(node, "SetNodeId", r);
    resp->success = (st == Status::kOk);
    resp->message = r.message;
    /* node_id 变了 → 我们内存里的寻址也变了：必须重新 configure 才能继续用。 */
    if (st == Status::kOk) {
        resp->message += " | re-run configure (node_id changed; the in-memory addressing is stale)";
        resp->advice = static_cast<std::uint8_t>(Advice::kCheckBusConfig);
    } else {
        resp->advice = static_cast<std::uint8_t>(r.advice);
    }
}

void JrBusServices::on_fault_reset(const jr_interfaces::srv::FaultReset::Request::SharedPtr req,
                                   jr_interfaces::srv::FaultReset::Response::SharedPtr resp)
{
    std::string err;
    std::vector<unsigned> sel;
    if (!select(req->joints, &sel, &err)) {
        resp->success = false;
        resp->message = err;
        return;
    }
    Guard g;
    if (!open_window(&g, sel, false, &err)) {
        resp->success = false;
        resp->message = err;
        return;
    }
    Result r;
    Status st = bus().fault_reset(sel.data(), static_cast<unsigned>(sel.size()), &r);
    std::string note;

    /* 实测（fw 1545）：estop 锁存清不掉 → 只有设备复位/断电。允许时替客户走一步。 */
    if (st != Status::kOk && r.advice == Advice::kNeedsDeviceReset && req->allow_device_reset) {
        Result r2;
        const Status st2 = bus().reset_device(sel.data(), static_cast<unsigned>(sel.size()), &r2);
        if (st2 == Status::kOk) {
            st = Status::kOk;
            reconfigure_after_reset(&note);
            r.set(Status::kOk, Advice::kNone,
                  "%s | device soft-reset performed (CLEAR_ERRORS could not clear the latch) | %s",
                  r2.message, note.c_str());
        }
    }
    log_result(node, "FaultReset", r);
    fill_results_same(&resp->results, sel, st, r.message);
    resp->success = (st == Status::kOk);
    resp->message = r.message;
    resp->advice = static_cast<std::uint8_t>(r.advice);
}

void JrBusServices::on_jog(const jr_interfaces::srv::Jog::Request::SharedPtr req,
                           jr_interfaces::srv::Jog::Response::SharedPtr resp)
{
    if (!req->confirm) {
        resp->success = false;
        resp->message = "Jog moves the joint: pass confirm=true (DESIGN §6.2/§8.5)";
        resp->exit_reason = "refused: confirm=false";
        return;
    }
    if (!(req->duration_s > 0.0) || req->duration_s > kJogMaxDurationS) {
        resp->success = false;
        resp->message = "duration_s must be in (0, 10] seconds (hard limit, no silent truncation)";
        resp->exit_reason = "refused: duration out of range";
        return;
    }
    const int idx = node->local_index_of(req->joint);
    if (idx < 0) {
        resp->success = false;
        resp->message = "joint '" + req->joint + "' is not on bus '" + std::string(bus_cfg().name) + "'";
        resp->exit_reason = "refused: unknown joint";
        return;
    }

    std::vector<unsigned> sel{static_cast<unsigned>(idx)};
    std::string err;
    /* 点动结束一定会走"hold → 等 2 周期 → 失能"：所以**不**恢复使能（安全默认）。 */
    Guard g;
    if (!open_window(&g, sel, false, &err)) {
        resp->success = false;
        resp->message = err;
        resp->exit_reason = "refused: " + err;
        return;
    }

    JointTarget target;
    target.position = req->position;
    target.kp = req->kp;
    target.kd = req->kd;
    target.torque = req->torque;
    target.si_gain = false;   /* Jog 沿用 CLI 的语义：kp/kd 是**线上值**（文档已注明） */

    const auto t0 = std::chrono::steady_clock::now();
    Result r;
    const Status st = bus().jog(static_cast<unsigned>(idx),
                                target, static_cast<unsigned>(req->duration_s * 1000.0), true, &r);
    const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    log_result(node, "Jog", r);

    resp->success = (st == Status::kOk);
    resp->message = r.message;
    resp->actual_duration_s = elapsed;
    resp->exit_reason = r.ok() ? (r.message[0] != '\0' ? r.message : "completed") : "failed";
}

/* ==========================================================================
 * 参数 / 描述符 / 诊断 / 产线
 * ======================================================================== */

void JrBusServices::on_read_params(const jr_interfaces::srv::ReadParams::Request::SharedPtr req,
                                   jr_interfaces::srv::ReadParams::Response::SharedPtr resp)
{
    std::string err;
    std::vector<unsigned> sel;
    if (!select(req->joints, &sel, &err)) {
        resp->success = false;
        resp->message = err;
        return;
    }
    if (req->paths.empty()) {
        resp->success = false;
        resp->message = "paths must not be empty (pass full endpoint paths, e.g. axis0.motor.config.gear_ratio)";
        return;
    }

    Guard g;
    if (!open_window(&g, sel, true, &err)) {
        resp->success = false;
        resp->message = err;
        return;
    }

    /* 关节 × 路径 的叉乘（joint 为主序）。路径字符串必须活到调用结束 → 先物化。 */
    std::vector<std::string> paths(req->paths.begin(), req->paths.end());
    std::vector<ParamReadItem> items;
    items.reserve(sel.size() * paths.size());
    for (unsigned j : sel) {
        for (std::size_t p = 0u; p < paths.size(); ++p) {
            ParamReadItem it;
            it.joint_index = j;
            it.path = paths[p].c_str();
            items.push_back(it);
        }
    }

    Result r;
    const Status st = bus().read_params(items.data(), static_cast<unsigned>(items.size()), &r);
    log_result(node, "ReadParams", r);

    std::size_t k = 0u;
    for (unsigned j : sel) {
        for (std::size_t p = 0u; p < paths.size(); ++p, ++k) {
            const ParamReadItem &it = items[k];
            jr_interfaces::msg::ParamValue v;
            v.joint = bus_cfg().joints[j].name;
            v.path = paths[p];
            v.type = type_code(it.declared_type);
            /* 按**声明类型**放值（规则写在 ParamValue.msg 里；不按"我们猜的类型"）。 */
            switch (it.declared_type) {
            case ParamType::kBool:
                v.bool_value = it.value.v.b;
                break;
            case ParamType::kU64:
                v.uint64_value = it.value.v.u64;
                break;
            case ParamType::kF32:
            case ParamType::kF64:
                v.double_value = it.value.as_double();
                break;
            case ParamType::kUnsupported:
                break;
            default:
                v.int64_value = it.value.as_i64();
                break;
            }
            if (it.declared_type != ParamType::kUnsupported) {
                char txt[64] = {};
                format_param_value(it.value, txt, sizeof(txt));
                v.text_value = txt;
            }
            v.status = st_code(it.status);
            v.message = it.message;
            resp->items.push_back(std::move(v));
        }
    }
    resp->success = (st == Status::kOk);
    resp->message = r.message;
}

void JrBusServices::on_write_params(const jr_interfaces::srv::WriteParams::Request::SharedPtr req,
                                    jr_interfaces::srv::WriteParams::Response::SharedPtr resp)
{
    /* ---- §8.5 写闸门 ---- */
    if (!req->confirm) {
        resp->success = false;
        resp->message = "parameter writes need confirm=true (DESIGN §8.5)";
        resp->all_verified = false;
        return;
    }
    if (!cfg().safety.allow_param_write) {
        resp->success = false;
        resp->message =
            "parameter writes are disabled (safety.allow_param_write=false, i.e. params.allow_write)";
        resp->all_verified = false;
        return;
    }
    if (req->persist && !cfg().safety.allow_flash_persist) {
        resp->success = false;
        resp->message = "persist=true requires params.allow_flash_persist=true";
        resp->all_verified = false;
        return;
    }
    if (req->writes.empty()) {
        resp->success = false;
        resp->message = "writes must not be empty";
        resp->all_verified = false;
        return;
    }

    std::string err;
    std::vector<unsigned> sel;
    for (const auto &w : req->writes) {
        unsigned idx = 0u;
        if (w.joint.empty()) {
            /* `joint` 是可选的（`jr_ctl write` 不传 `--joint` 时就是这个情况）：
               只有**唯一**关节的总线才能靠"只有一个"解析；多关节时必须点名 ——
               宁可拒绝，也不猜"用户想改哪一个"（写错关节在真机上是会动电机的）。 */
            if (bus_cfg().joint_count != 1u) {
                resp->success = false;
                resp->message =
                    "write: 'joint' is empty but this bus has " +
                    std::to_string(bus_cfg().joint_count) + " joints — name the joint explicitly";
                resp->all_verified = false;
                return;
            }
        } else {
            const int i = node->local_index_of(w.joint);
            if (i < 0) {
                resp->success = false;
                resp->message =
                    "joint '" + w.joint + "' is not on bus '" + std::string(bus_cfg().name) + "'";
                resp->all_verified = false;
                return;
            }
            idx = static_cast<unsigned>(i);
        }
        sel.push_back(idx);
    }

    Guard g;
    if (!open_window(&g, sel, true, &err)) {
        resp->success = false;
        resp->message = err;
        resp->all_verified = false;
        return;
    }

    /* 物化路径与（可选的）文本解析结果：`ParamWriteItem.path` 只存指针。 */
    std::vector<std::string> paths;
    std::vector<ParamWriteItem> items;
    std::vector<std::string> parse_errors;
    paths.reserve(req->writes.size());
    items.reserve(req->writes.size());
    parse_errors.resize(req->writes.size());

    for (std::size_t i = 0u; i < req->writes.size(); ++i) {
        const jr_interfaces::msg::ParamWrite &w = req->writes[i];
        paths.emplace_back(w.path);
        ParamWriteItem it;
        it.joint_index = sel[i];
        it.path = paths.back().c_str();
        it.declared_type = static_cast<ParamType>(w.type);
        it.requested.type = static_cast<ParamType>(w.type);

        if (!w.text_value.empty()) {
            /* ① 文本路径（推荐）：先查描述符拿到**声明类型**，再按它解析（超值域就拒）。 */
            EndpointInfo ep;
            Result lr;
            if (bus().lookup_endpoint(it.path, &ep, &lr) != Status::kOk) {
                /* ⚠⚠ 这里**绝不能**静默退回 `kUnsupported`：上层会报“unsupported parameter
                   type”，而真实原因（端点不在表里 / 总线没打开 / 描述符还没解析完…）被丢掉
                   ⇒ 排障被带偏（实测踩过，§13.3-39）。宁可在这里就把原因说清楚。 */
                parse_errors[i] = std::string("cannot use a text value: '") + paths.back() +
                                  "' is not resolvable in the endpoint table (" + lr.message +
                                  "). Pass a number together with an explicit type, or fix the "
                                  "path/descriptor first.";
            } else {
                it.declared_type = ep.type;
                char e[128] = {};
                it.requested.type = it.declared_type;
                if (parse_param_text(it.declared_type, w.text_value.c_str(), &it.requested, e,
                                     sizeof(e)) != Status::kOk) {
                    parse_errors[i] = e;
                }
            }
        } else {
            /* ② 类型化路径：按调用方给的类型装箱。 */
            switch (it.requested.type) {
            case ParamType::kBool:
                it.requested.v.b = w.bool_value;
                break;
            case ParamType::kU8:
                it.requested.v.u8 = static_cast<std::uint8_t>(w.int64_value);
                break;
            case ParamType::kI8:
                it.requested.v.i8 = static_cast<std::int8_t>(w.int64_value);
                break;
            case ParamType::kU16:
                it.requested.v.u16 = static_cast<std::uint16_t>(w.int64_value);
                break;
            case ParamType::kI16:
                it.requested.v.i16 = static_cast<std::int16_t>(w.int64_value);
                break;
            case ParamType::kU32:
                it.requested.v.u32 = static_cast<std::uint32_t>(w.int64_value);
                break;
            case ParamType::kI32:
                it.requested.v.i32 = static_cast<std::int32_t>(w.int64_value);
                break;
            case ParamType::kU64:
                it.requested.v.u64 = w.uint64_value;
                break;
            case ParamType::kI64:
                it.requested.v.i64 = w.int64_value;
                break;
            case ParamType::kF32:
                it.requested.v.f32 = static_cast<float>(w.double_value);
                break;
            case ParamType::kF64:
                it.requested.v.f64 = w.double_value;
                break;
            default:
                parse_errors[i] = "type must be given (TYPE_*) or use text_value (recommended)";
                break;
            }
        }
        items.push_back(it);
    }

    Result r;
    bool any_parse_error = false;
    for (const std::string &e : parse_errors) {
        if (!e.empty()) any_parse_error = true;
    }
    Status call_st = Status::kInvalidState;
    if (!any_parse_error) {
        call_st = bus().write_params(items.data(), static_cast<unsigned>(items.size()), true,
                                     req->persist, &r);
        log_result(node, "WriteParams", r);
    } else {
        r.set(Status::kInvalidArgument, Advice::kNone,
              "at least one write could not be prepared (see the per-item status)");
    }

    bool all_verified = true;
    for (std::size_t i = 0u; i < items.size(); ++i) {
        const ParamWriteItem &it = items[i];
        jr_interfaces::msg::ParamWriteResult out;
        out.joint = bus_cfg().joints[it.joint_index].name;
        out.path = paths[i];
        out.type = type_code(it.declared_type);
        /* 按**类型**取值（union 里读错成员是 UB；显式 switch 也顺便把规则写清）。 */
        switch (it.declared_type) {
        case ParamType::kBool:
            out.requested_bool = it.requested.v.b;
            out.value_bool = it.value.v.b;
            break;
        case ParamType::kU64:
            out.requested_uint64 = it.requested.v.u64;
            out.value_uint64 = it.value.v.u64;
            break;
        case ParamType::kF32:
        case ParamType::kF64:
            out.requested_double = it.requested.as_double();
            out.value_double = it.value.as_double();
            break;
        case ParamType::kUnsupported:
            break;
        default:
            out.requested_int64 = it.requested.as_i64();
            out.value_int64 = it.value.as_i64();
            break;
        }
        out.verified = it.verified;
        out.consumed_by_firmware = it.consumed_by_firmware;
        out.persisted = it.persisted;
        if (!parse_errors[i].empty()) {
            out.status = st_code(Status::kInvalidArgument);
            out.message = parse_errors[i];
        } else {
            out.status = st_code(it.status);
            out.message = it.message;
        }
        if (!out.verified) all_verified = false;
        resp->results.push_back(std::move(out));
    }
    (void)call_st;
    resp->all_verified = all_verified;
    /* ⚠ `success` 的口径（写在 srv 注释里）：全部项 status==OK。
       `consumed_by_firmware` 的端点 verified 必为 false 但 status 仍 OK → **不能**
       用 success 推断"值已验证"；要判这个请看 `all_verified`。 */
    resp->success = true;
    for (const auto &res : resp->results) {
        if (res.status != st_code(Status::kOk)) resp->success = false;
    }
    resp->message = r.message;
}

void JrBusServices::on_list_endpoints(const jr_interfaces::srv::ListEndpoints::Request::SharedPtr req,
                                      jr_interfaces::srv::ListEndpoints::Response::SharedPtr resp)
{
    std::string err;
    if (!check_bus(req->bus, &err)) {
        resp->success = false;
        resp->message = err;
        return;
    }
    const unsigned cap = (req->max == 0u) ? kEndpointCap : std::min(req->max, kEndpointCap);
    std::vector<EndpointInfo> eps(cap);
    unsigned count = 0u;
    Result r;
    /* 枚举只读内存里的描述符（不碰设备）→ 不需要暂停窗口。 */
    const Status st = bus().list_endpoints(req->filter.c_str(), eps.data(), cap, &count, &r);
    log_result(node, "ListEndpoints", r);
    for (unsigned i = 0u; i < count && i < cap; ++i) {
        jr_interfaces::msg::EndpointInfo m;
        m.path = eps[i].path;
        m.id = eps[i].id;
        m.type = type_code(eps[i].type);
        m.access = eps[i].access;
        m.readable = eps[i].readable();
        m.writable = eps[i].writable();
        resp->endpoints.push_back(std::move(m));
    }
    resp->success = (st == Status::kOk);
    resp->truncated = (count == cap);
    resp->message = r.message;
    if (resp->truncated) {
        resp->message += " | output truncated at max=" + std::to_string(cap) + " (narrow the filter)";
    }
}

void JrBusServices::on_lookup_endpoint(const jr_interfaces::srv::LookupEndpoint::Request::SharedPtr req,
                                       jr_interfaces::srv::LookupEndpoint::Response::SharedPtr resp)
{
    std::string err;
    if (!check_bus(req->bus, &err)) {
        resp->success = false;
        resp->message = err;
        return;
    }
    EndpointInfo ep;
    Result r;
    const Status st = bus().lookup_endpoint(req->path.c_str(), &ep, &r);
    resp->success = (st == Status::kOk);
    resp->found = (st == Status::kOk);
    resp->message = r.message;
    if (resp->found) {
        resp->endpoint.path = ep.path;
        resp->endpoint.id = ep.id;
        resp->endpoint.type = type_code(ep.type);
        resp->endpoint.access = ep.access;
        resp->endpoint.readable = ep.readable();
        resp->endpoint.writable = ep.writable();
    }
}

void JrBusServices::on_get_device_info(const jr_interfaces::srv::GetDeviceInfo::Request::SharedPtr req,
                                       jr_interfaces::srv::GetDeviceInfo::Response::SharedPtr resp)
{
    std::string err;
    if (!check_bus(req->bus, &err)) {
        resp->success = false;
        resp->message = err;
        return;
    }
    unsigned idx = 0u;
    if (!req->joint.empty()) {
        const int i = node->local_index_of(req->joint);
        if (i < 0) {
            resp->success = false;
            resp->message = "joint '" + req->joint + "' is not on bus '" + std::string(bus_cfg().name) + "'";
            return;
        }
        idx = static_cast<unsigned>(i);
    }
    const rt::DeviceInfoPOD d = bus().report().device[idx];
    resp->info.hw_version = d.hw_version;
    resp->info.fw_version = d.fw_version;
    resp->info.serial = d.serial;
    resp->info.classic = d.classic;
    resp->info.valid = d.valid;
    resp->success = d.valid;
    resp->message = d.valid ? "from the last QUERY_DEVICE_INFO (configure time)"
                            : "device info was never read (configure failed or the device did not answer)";
}

void JrBusServices::on_get_bus_stats(const jr_interfaces::srv::GetBusStats::Request::SharedPtr req,
                                     jr_interfaces::srv::GetBusStats::Response::SharedPtr resp)
{
    std::string err;
    if (!check_bus(req->bus, &err)) {
        resp->success = false;
        resp->message = err;
        return;
    }
    const StateSnapshot *s = tg()->acquire_snapshot();
    if (s == nullptr || node->bus_index_ >= s->bus_count) {
        resp->success = false;
        resp->message = "no snapshot yet (the tick has not produced one)";
        return;
    }
    /* 与话题同一套投影：这里复用话题填充逻辑会让代码重复 → 用同一个私有函数（见 jr_bus_node.cpp）。 */
    node->fill_bus_status(*s, &resp->status);
    resp->success = true;
    resp->message = "snapshot at tick " + std::to_string(s->rt.tick_count);
}

void JrBusServices::on_get_descriptor_info(
    const jr_interfaces::srv::GetDescriptorInfo::Request::SharedPtr req,
    jr_interfaces::srv::GetDescriptorInfo::Response::SharedPtr resp)
{
    std::string err;
    if (!check_bus(req->bus, &err)) {
        resp->success = false;
        resp->message = err;
        return;
    }
    const rt::DescInfoPOD &d = bus().report().desc;
    resp->info.bus = bus_cfg().name;
    resp->info.total_len = d.total_len;
    resp->info.json_bytes = d.json_bytes;
    resp->info.crc = d.crc;
    resp->info.fw_version = d.fw_version;
    resp->info.endpoint_count = d.endpoint_count;
    resp->info.parsed_total = d.parsed_total;
    resp->info.frames_rx = d.frames_rx;
    resp->info.complete = d.complete;
    resp->info.shared_hit = d.shared_hit;
    resp->info.from_cache = d.from_cache;

    char dir[kPathLen] = {};
    if (bus_cfg().desc.cache_dir[0] != '\0') {
        std::snprintf(dir, sizeof(dir), "%s", bus_cfg().desc.cache_dir);
    } else {
        DescCache::default_dir(dir, sizeof(dir));
    }
    char path[kPathLen * 2] = {};
    DescCache::path_of(dir, bus_cfg().name, path, sizeof(path));
    resp->info.cache_path = path;
    resp->success = d.complete;
    resp->message = d.complete ? (d.from_cache ? "loaded from cache" : "downloaded from the device")
                                : "descriptor is INCOMPLETE (download interrupted?) — cache was not written";
}

void JrBusServices::on_export_descriptor(
    const jr_interfaces::srv::ExportDescriptor::Request::SharedPtr req,
    jr_interfaces::srv::ExportDescriptor::Response::SharedPtr resp)
{
    std::string err;
    if (!check_bus(req->bus, &err)) {
        resp->success = false;
        resp->message = err;
        return;
    }
    const std::vector<std::uint8_t> &json = bus().descriptor_json();
    if (json.empty()) {
        resp->success = false;
        resp->message =
            "no raw descriptor JSON is available in this process (the descriptor came from the SDK "
            "internal cache or was never downloaded) — re-configure with cache_enabled=false";
        return;
    }
    resp->data.assign(json.begin(), json.end());
    resp->crc = bus().report().desc.crc;
    resp->fw_version = bus().report().desc.fw_version;
    resp->success = true;
    resp->message = "raw JSON (" + std::to_string(json.size()) + " bytes) — import with ImportDescriptor";
}

void JrBusServices::on_import_descriptor(
    const jr_interfaces::srv::ImportDescriptor::Request::SharedPtr req,
    jr_interfaces::srv::ImportDescriptor::Response::SharedPtr resp)
{
    if (!req->confirm) {
        resp->success = false;
        resp->message = "ImportDescriptor replaces the endpoint table: pass confirm=true (DESIGN §8.5)";
        return;
    }
    /* ⚠ `persist` **不支持**（SDK 没有任何“把描述符写进设备”的 API，§13.4）。
       静默忽略会让客户以为“已经预烧进设备了”—— 所以这里**明确拒绝**。 */
    if (req->persist) {
        resp->success = false;
        resp->device_reconfigured = false;
        resp->message =
            "persist=true is not supported: this SDK version has no API to write a descriptor into "
            "the device (the descriptor is owned by the device firmware). Import only affects this "
            "process (endpoint table + local cache). See DESIGN §13.4.";
        return;
    }
    if (req->data.empty()) {
        resp->success = false;
        resp->message = "data must not be empty (pass the bytes from ExportDescriptor)";
        return;
    }
    std::string err;
    if (!check_bus(req->bus, &err)) {
        resp->success = false;
        resp->message = err;
        return;
    }
    Guard g;
    if (!open_window(&g, {}, false, &err)) {
        resp->success = false;
        resp->message = err;
        return;
    }
    Result r;
    bool needs_reconfig = false;
    /* hint 必需（SDK 的 `!hint ⇒ INVALID_ARG`）：把 ExportDescriptor 的 crc/fw 原样带回；
       两者为 0 也**要传结构体**（“未知”是合法值，“没有”不是）。 */
    const jr::rt::DescHintPOD hint{static_cast<std::uint16_t>(req->crc),
                                   static_cast<std::uint32_t>(req->fw_version)};
    const Status st =
        bus().import_descriptor(req->data.data(), req->data.size(), hint, &r, &needs_reconfig);
    log_result(node, "ImportDescriptor", r);
    resp->crc = hint.crc;
    resp->data_crc32 = DescCache::crc32(req->data.data(), req->data.size());

    /* ⚠ 不只是“成功才重新 configure”：**失败的导入会触发回滚**（核心库的护栏，
       SDK 的失败导入会摧毁描述符）⇒ 回滚之后同样必须重新解析，否则端点表会一直是空的。 */
    if (needs_reconfig) {
        Result r2;
        const Status st2 = tg()->configure_buses(&r2);
        resp->device_reconfigured = (st2 == Status::kOk);
        resp->message = std::string(r.message) + (resp->device_reconfigured ? " | re-configured OK" : (" | re-configure FAILED: " + std::string(r2.message)));
        resp->success = (st == Status::kOk) && resp->device_reconfigured;
    } else {
        resp->device_reconfigured = false;
        resp->success = false;
        resp->message = r.message;
    }
}

void JrBusServices::on_heartbeat_hint(
    const jr_interfaces::srv::PublishHeartbeatHint::Request::SharedPtr req,
    jr_interfaces::srv::PublishHeartbeatHint::Response::SharedPtr resp)
{
    std::string err;
    if (!check_bus(req->bus, &err)) {
        resp->success = false;
        resp->message = err;
        return;
    }
    Guard g;
    if (!open_window(&g, {}, true, &err)) {
        resp->success = false;
        resp->message = err;
        return;
    }
    /* 读"现在的事实"（不读 configure 那一刻的缓存）。 */
    Result r;
    (void)bus().refresh_report(&r);

    const rt::JointInfoPOD &ji = bus().report().joint[0];
    const int idx = req->rate_ms == 0u ? 0 : 0;   /* 心跳是设备侧参数，逐关节一致；用第 0 个代表 */
    (void)idx;
    resp->current_rate_ms = ji.heartbeat_rate_ms;

    const std::uint32_t suggested =
        (req->rate_ms != 0u) ? req->rate_ms : (bus_cfg().heartbeat_ms != 0u ? bus_cfg().heartbeat_ms : 5u);
    resp->suggested_rate_ms = suggested;

    /* 影响评估：用**同一个预算模型**算，不拍脑袋（§7.1）。 */
    BusCfg probe = bus_cfg();
    probe.heartbeat_ms = suggested;
    const rt::BusPlanResult cur = bus().report().plan;
    const rt::BusPlanResult now = rt::plan_bus(probe, static_cast<double>(tg()->rate_hz()));
    resp->extra_frames_per_s = static_cast<std::uint32_t>(
        (now.rx_frames_per_sec > cur.rx_frames_per_sec) ? (now.rx_frames_per_sec - cur.rx_frames_per_sec) : 0.0);
    resp->bus_load_delta = static_cast<float>(now.load - cur.load);

    char impact[256] = {};
    std::snprintf(impact, sizeof(impact),
                  "heartbeat %u ms -> %u ms: +%u frame/s, load %.3f -> %.3f (%s, model-based estimate)",
                  resp->current_rate_ms, suggested, resp->extra_frames_per_s, cur.load, now.load,
                  now.feasible ? (now.warn ? "above the comfortable margin" : "within budget")
                               : "OVER the configured budget — do not do this");
    resp->impact = impact;
    resp->device_changed = false;   /* 本服务**不修改**设备（ADR-5：只给建议） */
    resp->success = true;
    resp->message =
        "hint only: nothing was changed on the device. Apply it with WriteParams "
        "(axis0.motor.config.heartbeat_rate_ms) + confirm, then optionally SaveConfig.";
}

/* ==========================================================================
 * 节点侧：服务挂载与查询
 * ======================================================================== */

int JrBusNode::local_index_of(const std::string &name) const noexcept
{
    const BusCfg *b = find_bus(cfg_, bus_name_.c_str());
    if (b == nullptr) return -1;
    for (unsigned j = 0u; j < b->joint_count; ++j) {
        if (name == b->joints[j].name) return static_cast<int>(j);
    }
    return -1;
}

bool JrBusNode::bus_matches(const std::string &req_bus, std::string *err) const
{
    if (req_bus.empty() || req_bus == bus_name_) return true;
    *err = "this node owns bus '" + bus_name_ + "', not '" + req_bus + "'";
    return false;
}

void JrBusNode::create_services()
{
    if (services_ != nullptr) return;   /* 幂等 */
    services_ = std::make_shared<JrBusServices>(this);
    services_->create();
}

void JrBusNode::destroy_services()
{
    if (services_ == nullptr) return;
    services_->destroy();
    services_.reset();
}

}  // namespace ros
}  // namespace jr
