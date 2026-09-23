# ============================================================================
#  JointROS 构建/测试镜像
# ----------------------------------------------------------------------------
#  设计取舍：
#   ① **不 COPY 源码**：源码通过 docker run -v 挂载进来（只读）+ 容器内拷到 /tmp 再编。
#      这样镜像与源码解耦（同一镜像可编任意版本），也避免把 Windows 侧
#      MinGW 的 build/ 与 Linux 的 ELF 产物混在同一个目录里。
#   ② **装 colcon/ament**：当前 `jr_core` 用纯 CMake 就能编，但 WP2/WP3 的 ROS 包
#      需要它们 —— 一次装好，后续不用再改镜像。
#   ③ **基础镜像可替换**：`--build-arg BASE_IMAGE=...` 换成国内镜像源即可
#      （docker.io 在国内经常拉不动）。
#
#  用法（在 WSL 里，见 docker/README.md）：
#     docker build -t jr-ros2:jazzy docker/
#     docker build -t jr-ros2:jazzy --build-arg BASE_IMAGE=<国内镜像源>/ros:jazzy-ros-base docker/
# ============================================================================
ARG BASE_IMAGE=ros:jazzy-ros-base
FROM ${BASE_IMAGE}

ARG DEBIAN_FRONTEND=noninteractive

# ⚠ 先 `apt-get upgrade`：基础镜像里的核心库可能比 apt 仓库旧一批，
# 混装会得到 ABI 不一致的运行时。实测（Lyrical）：
#   ros:lyrical-ros-base 的 libservice_msgs__rosidl_typesupport_fastrtps_c.so
#   不导出 has_buffer_fields_service_msgs__msg__ServiceEventInfo，
#   而仓库里的 controller_manager_msgs 链接了它 →
#   `ros2_control_node` / `python3` 启动即 `undefined symbol`（JTC 示例跑不起来）。
# 这一步让镜像与仓库对齐（一次构建、进镜像层缓存，不影响运行时）。
#
# 构建工具链 + ROS 2 构建/测试框架（与哈工大/清华源无关，全部走官方 apt）
RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        ninja-build \
        ccache \
        pkg-config \
        libyaml-cpp-dev \
        python3-pip \
        python3-colcon-common-extensions \
        ros-${ROS_DISTRO}-ament-cmake \
        ros-${ROS_DISTRO}-ament-cmake-gtest \
        ros-${ROS_DISTRO}-ament-lint-auto \
        ros-${ROS_DISTRO}-ament-lint-common \
        ros-${ROS_DISTRO}-diagnostic-updater \
        ros-${ROS_DISTRO}-ros2-control \
        ros-${ROS_DISTRO}-ros2-controllers \
        ros-${ROS_DISTRO}-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/*
# 说明：`ros2-control` 带 controller_manager + hardware_interface（WP3 的编译依赖）；
# `ros2-controllers` 带 joint_trajectory_controller / joint_state_broadcaster
# （WP3 验收要"跑通 JTC 示例"）；
# `robot-state-publisher` 用来把 URDF 发到 `/robot_description` 话题 ——
# **Jazzy 的 controller_manager 从这个话题取模型**（Humble 读同名参数，两个都兼容）。
# `libyaml-cpp-dev` 用于我们自己解析配置 YAML
# （与 WP2 节点/工具共用同一份配置文件格式）。

# 容器内不写任何东西到挂载的源码树：构建目录固定在 /tmp。
ENV JR_BUILD_DIR=/tmp/jr-build
ENV JR_COLCON_BUILD_BASE=/tmp/jr-colcon/build
ENV JR_COLCON_INSTALL_BASE=/tmp/jr-colcon/install
ENV JR_COLCON_LOG_BASE=/tmp/jr-colcon/log
ENV JR_CCACHE_DIR=/tmp/jr-ccache

# 默认入口：由 run.sh 决定跑什么（build_and_test）。
WORKDIR /jr
CMD ["/bin/bash"]
