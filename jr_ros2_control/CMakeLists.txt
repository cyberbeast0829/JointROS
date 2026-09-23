# ============================================================================
#  jr_ros2_control —— ros2_control SystemInterface（WP3，**P0 最优先**）
# ----------------------------------------------------------------------------
#  依赖写法遵循 DESIGN §9.2：统一 `find_package` + `target_link_libraries`，
#  不用 `ament_target_dependencies`（新发行版正在退场）。
# ============================================================================
cmake_minimum_required(VERSION 3.16)
# C 一并声明：本包经 jr_core 使用 JointSDK（C 库）；CMake 4.x 下漏了会直接报
# `No known features for C compiler`（见 jr_ros2/CMakeLists.txt 里的说明）。
project(jr_ros2_control C CXX)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# JointSDK：本包通过 `jr_core` **间接**依赖它。
# ⚠ 为什么这里也要 locate 一次（踩过）：SDK 有两种获取方式 ——
#   ① 已安装：`find_package(jsdk_can)` → `jr_ros2` 会把它导出，消费者自动拿到；
#   ② 源码模式（-DJRSDK_SOURCE_DIR=...）：SDK 目标在**上游自己的构建树**里、投不出去，
#      所以消费者必须自己 locate 一次，否则 `find_package(jr_ros2)` 会报
#      "The following imported targets are referenced, but are missing: jsdk::can"。
#   `jr_locate_jsdk()` 在两种模式下都正确（源码模式会 add_subdirectory 到本包构建树）。
set(_jr_cmake_dir "${CMAKE_CURRENT_SOURCE_DIR}/../cmake")
if(EXISTS "${_jr_cmake_dir}/jr_sdk.cmake")
    list(APPEND CMAKE_MODULE_PATH "${_jr_cmake_dir}")
    include(jr_sdk)
    jr_locate_jsdk()
else()
    find_package(jsdk_can REQUIRED)
endif()

# jr_ros2 提供两个目标：jr_core（零依赖 RT 核心）与 jr_config_yaml（YAML 配置加载）。
find_package(jr_ros2 REQUIRED)

# ============================================================================
#  发行版差异（`on_init` 的签名）**不在 CMake 里探测**
# ----------------------------------------------------------------------------
#  历史教训（真踩过）：这里曾经用 `check_cxx_source_compiles` 探测
#  "旧签名还能不能 override"，只给了迷你工程 `hardware_interface` 目标自己的
#  include 目录 → 拿不到传递依赖的头（缺 `rcpputils/pointer_traits.hpp`）→
#  探测在三个发行版上**全部失败**，且失败时变量是**空串**（不是 FALSE）→
#  `#if` 把空串当 0 → 全部走了新签名分支，Humble 上才爆出来。
#  现在判定放在编译器手里（`compat/distro_compat.hpp` 里 `__has_include` +
#  静态断言）：与真实编译**同条件**，且判定错了会**立刻编译失败**。
# ============================================================================

add_library(jr_ros2_control SHARED
    src/jr_system_interface.cpp
    src/compat/distro_compat.cpp)

target_include_directories(jr_ros2_control PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(jr_ros2_control PUBLIC
    hardware_interface::hardware_interface
    pluginlib::pluginlib
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    # ⚠ 必须用**带命名空间**的名字：ament 导出时给目标加上了 `<pkg>::` 前缀
    #   （见安装出来的 jr_coreExport.cmake 里的 `IN ITEMS jr_ros2::jr_core`）。
    #   写成裸 `jr_core` 不会报错，但拿不到 include 目录 → 症状是编译时
    #   "fatal error: jr_ros2/jr_command.hpp: No such file or directory"。
    jr_ros2::jr_core
    jr_ros2::jr_config_yaml)

# 警告：与核心库同风格（本仓库的规矩：新代码不允许带告警进来）。
if(MSVC)
    target_compile_options(jr_ros2_control PRIVATE /W4 /utf-8)
else()
    target_compile_options(jr_ros2_control PRIVATE -Wall -Wextra -Wpedantic -Wshadow -Wconversion)
endif()

# pluginlib：让 controller_manager 能按名字加载本组件。
pluginlib_export_plugin_description_file(hardware_interface jr_ros2_control.xml)

# ---- 测试：**不依赖 controller_manager** 地验证组件行为 ----
# 为什么这么做：CM + JTC 的端到端很重（URDF、控制器配置、时序），定位问题慢。
# 组件的核心契约（接口集合、两种 tick 来源、生命周期、安全落点）可以也应该被单测钉住；
# "跑通 JTC 示例"是**额外的**集成验证，不替代这里。
if(BUILD_TESTING)
    set(_jr_test_dir "${CMAKE_CURRENT_SOURCE_DIR}/../jr_ros2/test")
    if(EXISTS "${_jr_test_dir}/jr_test.hpp")
        add_executable(test_jr_system_interface test/test_jr_system_interface.cpp)
        target_include_directories(test_jr_system_interface PRIVATE "${_jr_test_dir}")
        target_link_libraries(test_jr_system_interface PRIVATE jr_ros2_control)
        if(MSVC)
            target_compile_options(test_jr_system_interface PRIVATE /W4 /utf-8)
        else()
            target_compile_options(test_jr_system_interface PRIVATE -Wall -Wextra)
        endif()
        add_test(NAME test_jr_system_interface COMMAND test_jr_system_interface)
        set_tests_properties(test_jr_system_interface PROPERTIES TIMEOUT 120)
    else()
        message(STATUS "jr_ros2_control: sibling jr_ros2/test not found -> component test skipped")
    endif()
endif()

# ---- 安装 / 导出 ----
install(TARGETS jr_ros2_control
        EXPORT export_jr_ros2_control
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_jr_ros2_control HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle jr_ros2)

ament_package()
