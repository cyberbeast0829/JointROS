/**
 * @file    jr_config_yaml.hpp
 * @brief   YAML → `jr::Config` 加载器（schema 见 DESIGN §6.4）
 *
 * @par 为什么单独一个目标（`jr_config_yaml`）而不是塞进 `jr_core`
 *  `jr_core` 是**零依赖**的实时核心：只要有编译器 + CMake 就能编、能测
 *  （Windows/MinGW、客户交叉编译环境都是这么验证的）。YAML 解析要 yaml-cpp，
 *  不该把这个依赖传染给它。所以：`jr_core`（无依赖）+ `jr_config_yaml`（+yaml-cpp）。
 *
 * @par 为什么三处（节点 / ros2_control / 工具）都用同一份文件格式
 *  客户只维护**一份**配置文件：WP2 的节点、WP3 的 ros2_control 组件、WP4 的
 *  `jr_ctl`/`jr_hw_verify` 都读它。两套 schema 迟早漂移（本项目已有过
 *  "两份 SDK 状态码映射自己漂了" 的教训）。
 */

#ifndef JR_ROS2_JR_CONFIG_YAML_HPP
#define JR_ROS2_JR_CONFIG_YAML_HPP

#include <cstdint>
#include <cstring>

#include "jr_ros2/jr_config.hpp"
#include "jr_ros2/jr_status.hpp"

namespace jr {

/** 节点层配置：属于**驱动节点**、核心库不消费的键（DESIGN §6.4）。
 *
 *  为什么在这里解析（而不是让节点自己再读一遍 YAML 或另开一套 ROS 参数）：
 *  客户只维护**一份**配置文件。两份解析迟早漂移 —— 本项目已经有
 *  "两份 SDK 状态码映射自己漂了" 的教训。
 *  同时这些键仍然会被列进 `YamlLoadReport`（日志/诊断里能看到"哪些键属于节点"）。 */
struct NodeCfg {
    std::uint32_t publish_hz = 500u;      /**< `joint_feedback` 发布率（§6.3 默认 500 Hz） */
    std::uint32_t joint_state_hz = 100u;  /**< `sensor_msgs/JointState` 发布率 */
    /** F11：**端点轮询**周期（ms；0 = 关闭，默认）。
     *
     *  与 `buses[].poll_period_ms` 是**两回事**：那个是 SDK 的 `unicast_poll` 反馈策略
     *  （真机实测**不刷新**上报帧、只把开销放大 165 倍 ⇒ 已弃用，见 DESIGN §13.3-53）；
     *  这个是我们自己按周期去**读设备端点** `pos_estimate`/`vel_estimate`，把读回来的真值
     *  用来覆盖 `/joint_feedback` 里的位置/速度。
     *
     *  ⚠ 只在**本条总线上所有关节都失能**时才轮询：读参数要进安全暂停窗口，而
     *  `TickGroup::pause()` 的语义是**先安全失能**（见 "rt/jr_ops.hpp" 注释）——
     *  拿它做 10 Hz 轮询会变成每秒失能/使能十次。在动的关节不会因为开了这个键被打断。 */
    std::uint32_t endpoint_poll_ms = 0u;   /* DEPRECATED (v0.20)：用 state_request_ms 取代 */

    /** F31：**非阻塞状态请求**周期（ms；0 = 关闭，默认）。
     *
     *  开着它会在 tick 的 `cycle_end()` 之后按周期发 `QUERY_POS_VEL`(0x41)
     *  （`jsdk_joint_request_state()`：**发出即返回、不与 tick 互斥**），应答由 SDK 的收帧
     *  路径回填到反馈缓存 ⇒ “驱动中的新鲜位置/速度”不再需要暂停 tick 去读端点。
     *
     *  ⚠ 0x41/0x44 **不喂设备看门狗**（SDK 头文件原话）⇒ 只能**额外**发，不能顶替控制帧。
     *  ⚠ 建议 **≤10 Hz/关节**（SDK 头文件原话）：一次请求占 2 帧的总线时间。
     *  ⚠ 与 `endpoint_poll_ms` 不同：那个要进安全暂停窗口（会先失能关节），只适合待机时
     *  交叉核对；这个可以在**驱动中**用。 */
    std::uint32_t state_request_ms = 0u;
};

/** 加载结论（比 `Result` 多两项**必须让用户看到**的事实）。 */
struct YamlLoadReport {
    /** `validate_config()` 的非致命结论（例如"有 node_id > 7 → 无法广播寻址"）。 */
    ConfigNotes notes = {};

    /** 节点层键的**取值**（已解析）。 */
    NodeCfg node = {};

    /** 本次出现的**节点层**键个数（与 `node_keys_text` 配对）。
     *
     *  为什么要报出来：客户写了键却没人用 → 他会以为生效了。
     *  这里把键名列出来（取值在 `node` 里），日志/诊断一眼能看出归属。 */
    unsigned node_keys = 0u;
    char     node_keys_text[192] = {};
};

/**
 * 从 YAML 文件读配置并**当场校验**（`validate_config()`）。
 *
 * @param path  YAML 路径（支持 `jr:` 直接挂参数，也支持 ROS 参数文件风格
 *              的 `jr: ros__parameters:` 包一层）。
 * @param out   输出（成功时才可信）。
 * @param res   失败原因（含**键路径**，例如 `buses[0].joints: unknown joint 'FL_hip3'`）。
 * @param rep   可空；非空时填充"未消费的节点层键"。
 *
 * @retval kOk           成功（且 `validate_config()` 通过）
 * @retval kNotFound     文件打不开
 * @retval kInvalidArgument 语法/键名/取值错误（**未知键一律报错**，不静默忽略）
 */
Status load_config_yaml(const char *path, Config *out, Result *res,
                        YamlLoadReport *rep = nullptr) noexcept;

/** 同上，但从内存里的 YAML 文本解析（测试与"参数下发"场景用）。 */
Status parse_config_yaml(const char *yaml_text, const char *what, Config *out, Result *res,
                         YamlLoadReport *rep = nullptr) noexcept;

}  // namespace jr

#endif /* JR_ROS2_JR_CONFIG_YAML_HPP */
