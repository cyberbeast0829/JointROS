# ============================================================================
#  JointROS 开发/CI 顶层构建（**不是** colcon 包：没有 package.xml，colcon 会忽略它）
# ----------------------------------------------------------------------------
#  用途：在没有 ROS 的机器上编译并测试 `jr_core`（含虚拟总线集成测试）。
#     cmake -S . -B build -G "MinGW Makefiles" -DJRSDK_SOURCE_DIR=/d/projects/cheetah/JointSDK
#     cmake --build build -j
#     ctest --test-dir build --output-on-failure
#  装了 ROS 的机器请直接用 colcon 构建各 `jr_*` 包。
# ============================================================================
cmake_minimum_required(VERSION 3.16)
project(jointros_dev LANGUAGES C CXX)

# ⚠ 必须在**顶层**调用 enable_testing()：否则 ctest 在 build 根目录找不到任何测试
#   （子目录里的 enable_testing() 只对子目录自己生效）——这个坑第一次构建就会撞上。
enable_testing()

add_subdirectory(jr_ros2)
