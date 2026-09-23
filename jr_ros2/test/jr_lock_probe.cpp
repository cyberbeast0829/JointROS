/**
 * @file    jr_lock_probe.cpp
 * @brief   §10.2 ⑦ 用的最小驱动：起一个 `jr_bus` 节点 → **configure** → 按需持锁 → 退出。
 *
 * @par 为什么要单独一个进程
 *  总线锁（ADR-9）是**进程间**的 `flock` —— 只有真的两个进程才测得到它。
 *  在同一个进程里建两个节点（`test_jr_bus_node` 的做法）永远测不出"第二个进程抢不到锁"，
 *  因为锁的持有者/释放语义都与进程生命周期绑定。
 *
 * @par 用法
 *  `jr_lock_probe <config.yaml> <node_name> [hold_file]`
 *   - `hold_file` 为空：configure 成功后立刻 cleanup + shutdown 退出（**释放锁**）；
 *   - `hold_file` 非空：configure 成功后**一直持锁**，直到该文件出现（脚本用来精确控制时序，
 *     不靠 sleep 猜）。
 *
 * @par 退出码（脚本按它判决，不解析日志猜成功与否）
 *   0 = configure 成功；3 = configure 失败（锁被别人占着就是这条，日志里有持有者 PID）。
 */

#include <cstdio>
#include <memory>
#include <string>
#include <thread>

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp/rclcpp.hpp>

#include "jr_ros2/ros/jr_bus_node.hpp"

namespace {

/** 有界等待一个文件出现（不 sleep 猜时长；上限到了就返回 false 让脚本判失败）。 */
bool wait_for_file(const std::string &path, unsigned timeout_ms)
{
    const unsigned step_ms = 50u;
    for (unsigned waited = 0u; waited < timeout_ms; waited += step_ms) {
        if (std::FILE *f = std::fopen(path.c_str(), "rb")) {
            std::fclose(f);
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(step_ms));
    }
    return false;
}

}  // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    if (argc < 3) {
        std::fprintf(stderr, "usage: jr_lock_probe <config.yaml> <node_name> [hold_file]\n");
        rclcpp::shutdown();
        return 2;
    }
    const std::string cfg = argv[1];
    const std::string name = argv[2];
    const std::string hold_file = (argc > 3) ? argv[3] : std::string();

    rclcpp::NodeOptions opts;
    opts.parameter_overrides({rclcpp::Parameter("config_file", cfg),
                              rclcpp::Parameter("bus", std::string("vbus")),
                              rclcpp::Parameter("use_sim_time", false)});
    opts.arguments({"--ros-args", "-r", std::string("__node:=") + name});
    auto node = std::make_shared<jr::ros::JrBusNode>(opts);

    const auto st = node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    const bool configured = st.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;

    if (configured && !hold_file.empty()) {
        std::printf("[probe] configured, holding the bus lock until '%s' appears\n", hold_file.c_str());
        std::fflush(stdout);
        wait_for_file(hold_file, 60000u);
    }

    /* 无论成功失败都走到 finalized（否则析构时会打 "not shut down" 的噪声）。 */
    if (configured) node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
    node.reset();
    rclcpp::shutdown();
    return configured ? 0 : 3;
}
