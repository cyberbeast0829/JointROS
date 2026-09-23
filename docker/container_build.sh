#!/usr/bin/env bash
# ============================================================================
#  容器**内部**的构建 + 测试脚本（由 docker/run.sh 调起）
# ----------------------------------------------------------------------------
#  输入（环境变量）：
#     JR_SRC_RO  = 挂载进来的源码根（只读，形如 /host/JointROS）
#     JR_SDK_RO  = 挂载进来的 JointSDK 源码（只读）
#     ROS_DISTRO = 发行版（镜像自带）
#  行为：
#     ① 把源码拷到 /tmp（不在挂载树上留任何产物）；
#     ② **纯 CMake 路径**：编 jr_core + 跑全部测试（这是本机 Windows 跑不了的部分：
#        Linux 的 flock / SCHED_FIFO / mlockall / clock_nanosleep 代码路径）；
#     ③ **colcon 路径**：若工作区里有 `jr_*` 包，则用 colcon 再编一遍（WP2/WP3 进来后
#        这条才是主路径）；
#     ④ 汇总：任一失败 → 退出码非 0。
# ============================================================================
set -euo pipefail

JR_SRC_RO="${JR_SRC_RO:-/host/JointROS}"
JR_SDK_RO="${JR_SDK_RO:-/host/JointSDK}"
EXTRA_CMAKE_ARGS="${JR_CMAKE_ARGS:-}"

echo "=============================================================="
echo " 容器内构建：ROS_DISTRO=${ROS_DISTRO:-<none>}"
gcc --version | head -1
cmake --version | head -1
echo "=============================================================="

# ---- ① 拷到 /tmp（源码树只读，产物不落挂载盘） ----
WS=/tmp/jr-ws
rm -rf "$WS"
mkdir -p "$WS"
cp -a "$JR_SRC_RO" "$WS/JointROS"
echo "[1/4] 源码已拷到 $WS/JointROS（挂载树保持只读，无污染）"

cd "$WS/JointROS"

# ---- ② 纯 CMake：jr_core + 全部测试（**刻意关掉 ament**） ----
# 为什么要显式关：ROS 基础镜像的 CMAKE_PREFIX_PATH 预置了 /opt/ros，`find_package(ament_cmake)`
# 必然命中 → 这条“无 ROS 路径”的验证就名存实亡了。两个模式分别验证：
#   ② = 无 ROS（客户的非 ROS 集成/交叉编译环境）  ③ = ament/colcon
# 另外它也是**必需**的：ament 模式下同一份 CMakeLists 还要求包资源索引等，
# 两条路径都要能编。
echo "[2/4] 纯 CMake 构建 + ctest（强制无 ROS：验证客户的非 ROS 集成路径）"
cmake -S . -B "$JR_BUILD_DIR" \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DCMAKE_DISABLE_FIND_PACKAGE_ament_cmake=ON \
      -DJRSDK_SOURCE_DIR="$JR_SDK_RO" \
      -DJR_WERROR=ON \
      ${EXTRA_CMAKE_ARGS}
cmake --build "$JR_BUILD_DIR" -j "$(nproc)"
ctest --test-dir "$JR_BUILD_DIR" --output-on-failure

# ---- ③ colcon：一旦有 jr_* 包（WP2/WP3）就走这条 ----
if compgen -G "jr_*/package.xml" > /dev/null; then
    echo "[3/4] colcon 构建 ROS 包"
    # ⚠ ROS 的 setup.bash 会引用未定义变量（AMENT_TRACE_SETUP_FILES 等），
    #    与本脚本的 `set -u` 直接冲突 —— 症状是 `line 8: AMENT_TRACE_SETUP_FILES:
    #    unbound variable` 然后本脚本以非 0 退出（而前面的 9/9 测试其实是全绿的）。
    #    ROS 官方也建议在 set -u 下这样包裹。
    set +u
    # shellcheck disable=SC1091
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    set -u
    # ⚠ `--log-base` 是 colcon 的**全局**选项，必须放在动词（build/test）**之前**：
    #    写成 `colcon build --log-base X` 会报 "unrecognized arguments"。
    # ⚠ 必须显式给 `--base-paths`：本仓库**根目录也有 CMakeLists.txt**（纯 CMake 开发
    #   构建入口），colcon 会把它识别成一个 `cmake` 类型包（名字取自 project()），
    #   **并且不再深入子目录** —— 后果是我们真正的 ament 包根本没被构建：
    #   `colcon list` 只输出 `jointros_dev  .  (cmake)`，ament_package() 没跑,
    #   包资源索引不存在（ros2 pkg list 里看不到），但测试却照样跑了 9 个
    #   （因为那个 dev 项目 add_subdirectory 了 jr_ros2）——不查日志根本发现不了。
    #   客户的标准做法是把 jr_* 包放进他们自己的工作区 src/（那里没这个干扰）；
    #   本脚本直接指向包目录，等价且更明确。
    colcon --log-base "$JR_COLCON_LOG_BASE" build \
        --base-paths jr_interfaces jr_ros2 jr_ros2_control jr_bringup \
        --build-base "$JR_COLCON_BUILD_BASE" \
        --install-base "$JR_COLCON_INSTALL_BASE" \
        --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DJRSDK_SOURCE_DIR="$JR_SDK_RO"
    set +u
    # shellcheck disable=SC1091
    source "$JR_COLCON_INSTALL_BASE/setup.bash"
    set -u
    # ⚠ 以前这里直接 `colcon test --return-code-on-test-failure`，一旦失败脚本立刻退出
    #   （set -e）→ **失败明细一行都看不到**。现在先记账、把 --verbose 明细打出来再退出。
    set +e
    colcon --log-base "$JR_COLCON_LOG_BASE" test \
        --base-paths jr_interfaces jr_ros2 jr_ros2_control jr_bringup \
        --build-base "$JR_COLCON_BUILD_BASE" \
        --install-base "$JR_COLCON_INSTALL_BASE" \
        --return-code-on-test-failure
    jr_test_rc=$?
    set -e
    if [[ ${jr_test_rc} -ne 0 ]]; then
        echo "================ 测试失败明细（colcon test-result --verbose）================"
        colcon test-result --test-result-base "$JR_COLCON_BUILD_BASE" --verbose || true
        echo "==========================================================================="
    fi
    colcon test-result --test-result-base "$JR_COLCON_BUILD_BASE" | tail -5
    [[ ${jr_test_rc} -eq 0 ]]
else
    echo "[3/4] 工作区里还没有 jr_* ROS 包（WP2/WP3 尚未落地）→ 跳过 colcon"
fi

# ---- ④ JTC 端到端（WP3 验收）：真起 controller_manager + JTC，走一条轨迹 ----
# 为什么放在这里而不是当单测：这条链路跨 CM → JTC → 我们的 SystemInterface → jr_core
# → JointSDK → 设备（虚拟）→ 反馈回读，只有真跑一遍才能证明“命令能闭环”。
if [[ -x "jr_ros2_control/test/jtc_demo/run.sh" ]]; then
    echo "[4/4] JTC 端到端（controller_manager + JointTrajectoryController，虚拟总线）"
    set +u
    # shellcheck disable=SC1091
    source "$JR_COLCON_INSTALL_BASE/setup.bash"
    set -u
    # 两种模式都跑：CSP 是消费/传统工业客户最常见的接法（位置环在驱动器里），
    # ⚠ 两种模式的**命令接口集不同** → URDF 声明、控制器 claim 都要跟着变；
    #   三者对不上时 controller_manager 会拒绝初始化硬件（见 run.sh 头注释）。
    for jr_demo_mode in mit csp; do
        echo "---- JTC demo：mode=${jr_demo_mode}"
        JR_DEMO_INSTALL="$JR_COLCON_INSTALL_BASE" \
            bash jr_ros2_control/test/jtc_demo/run.sh --mode "${jr_demo_mode}"
        # 发行版差异的**证据**：组件启动时会把自己实际用的 `on_init` 签名打进日志
        # （Humble = legacy，Jazzy/Lyrical = modern）。看一眼就知道这一发行版走的哪条路。
        grep -ho "on_init=[^,]*" /tmp/jr-jtc-demo/cm.log 2>/dev/null | head -1 \
            | sed 's/^/    [compat] /' || true
    done
else
    echo "[4/4] 没有 jtc_demo → 跳过"
fi

echo "=============================================================="
echo " 容器内全部通过 ✓"
echo "=============================================================="
