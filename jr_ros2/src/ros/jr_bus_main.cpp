/**
 * @file    jr_bus_main.cpp
 * @brief   `jr_bus` 可执行：一条总线一个 lifecycle 节点
 *
 * @par 用法（一个节点 = 一条总线，ADR-1）
 *  ros2 run jr_ros2 jr_bus --ros-args -r __node:=can0 \
 *      -p config_file:=/path/to/jr.yaml -p bus:=can0
 *
 * @par 为什么 main 里要显式做 lifecycle 关停
 *  Ctrl-C 之后 `spin()` 返回，如果只靠析构函数兜底，`on_shutdown` 里的安全退出序列
 *  （按 `safety.on_exit` 失能/锁位/泄力）就不会走完整流程 —— 而 §8.2 明确要求
 *  "SIGINT/SIGTERM 与正常退出走同一条路径"。所以这里主动触发对应的 shutdown 迁移。
 */

#include <memory>

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "jr_ros2/ros/jr_bus_node.hpp"

namespace {

/** 按当前 lifecycle 状态选对应的 shutdown 迁移（没有可走的迁移就什么都不做）。 */
void trigger_lifecycle_shutdown(const rclcpp_lifecycle::LifecycleNode::SharedPtr &node)
{
    const std::uint8_t s = node->get_current_state().id();
    std::uint8_t t = 0u;
    if (s == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        t = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN;
    } else if (s == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        t = lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN;
    } else if (s == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
        t = lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN;
    } else {
        return;   /* 已经是 FINALIZED */
    }
    node->trigger_transition(t);
}

}  // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    auto node = std::make_shared<jr::ros::JrBusNode>(options);

    /* 多线程执行器 + 回调组（§5.1 的 T2/T3/T4）：
       - 阻塞型服务（标定/参数）不能把状态发布与命令接收一起卡住；
       - 同组内仍然串行（命令处理不并发，避免 seq 与掩码乱序）。
       线程数 4 = 1（命令）+ 1（状态）+ 1（服务）+ 1 余量。 */
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 4u, false);
    exec.add_node(node->get_node_base_interface());
    exec.spin();
    exec.remove_node(node->get_node_base_interface());

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    /* 退出前把 lifecycle 走完（安全失能 → 关总线 → 释放锁），不依赖析构顺序。 */
    trigger_lifecycle_shutdown(node);

    node.reset();
    return 0;
}
