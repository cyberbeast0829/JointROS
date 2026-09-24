# ============================================================================
#  jr_ros2 —— 实时核心库（jr_core）+（后续）驱动节点与工具
# ----------------------------------------------------------------------------
#  两种构建模式（同一个 CMakeLists，避免"两份构建脚本会分叉"）：
#    ① colcon/ament 模式：检测到 ament_cmake → 用 ament 宏（ROS 侧目标后续加在
#       `if(JR_WITH_ROS)` 分支里）。
#    ② 纯 CMake 模式：没有 ROS 时也能**编译并测试核心库**（含虚拟总线集成测试），
#       这正是"在 Windows/WSL 上不装 ROS 也能验证实时逻辑"的关键。
# ============================================================================
cmake_minimum_required(VERSION 3.16)
# ⚠ 声明 C 不能省：JointSDK 是 C 库（经由 add_subdirectory 进来），
#   而 **CMake 4.x 把"没启用 C 却要用 C 特性"从警告升级成了硬错**：
#   `CMake Error in CMakeLists.txt: No known features for C compiler "" version .`
#   （在 Lyrical 容器里实际撞到：gcc 15 + CMake 4.2.3）。
project(jr_ros2 C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ⚠ 静态库必须带 PIC：`jr_ros2_control` 是**共享库**，静态库进 .so 时若没有
#   `-fPIC`，链接器会直接拒："relocation R_X86_64_PC32 against symbol ... can not be
#   used when making a shared object; recompile with -fPIC"。
#   设在这里是为了让下面的 add_subdirectory(JointSDK) 也继承到（SDK 的静态库同理）。
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake QUIET)
set(JR_WITH_ROS ${ament_cmake_FOUND})

if(JR_WITH_ROS)
    message(STATUS "jr_ros2: ament mode (ROS 2 build)")
else()
    message(STATUS "jr_ros2: standalone mode (core library + tests only)")
endif()

find_package(Threads REQUIRED)

include(CMakeDependentOption)
option(JR_BUILD_TESTS "Build jr_ros2 unit/integration tests" ON)
option(JR_WERROR "Treat warnings as errors" OFF)

# --- JointSDK（三种获取方式，见 cmake/jr_sdk.cmake）---
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake")
include(jr_sdk)
jr_locate_jsdk()

# 后端可用性宏：把"SDK 这次到底编了哪些后端"告诉我们的代码，避免链接到
# 不存在的工厂函数（Windows 上引用 socketcan 会直接链接失败）。
if(JR_SDK_MODE STREQUAL "subdirectory" OR JR_SDK_MODE STREQUAL "fetchcontent")
    set(_jr_has_virtual   ${JSDK_BUILD_HAL_VIRTUAL})
    set(_jr_has_slcan     ${JSDK_BUILD_HAL_SLCAN})
    set(_jr_has_socketcan ${JSDK_BUILD_HAL_SOCKETCAN})
    set(_jr_has_pcan      ${JSDK_BUILD_HAL_PCAN})
else()
    # 已安装的 SDK：无法探测 → 平台默认（用户可 -D 覆盖）
    set(_jr_has_virtual ON)
    set(_jr_has_slcan ON)
    if(UNIX AND NOT APPLE)
        set(_jr_has_socketcan ON)
        set(_jr_has_pcan OFF)
    else()
        set(_jr_has_socketcan OFF)
        set(_jr_has_pcan ON)
    endif()
    message(STATUS "jr: installed-SDK mode: assuming HAL availability "
                   "(virtual=${_jr_has_virtual} slcan=${_jr_has_slcan} "
                   "socketcan=${_jr_has_socketcan} pcan=${_jr_has_pcan}); override with -DJR_HAS_HAL_*=0/1")
endif()

foreach(_b virtual slcan socketcan pcan)
    if(NOT DEFINED JR_HAS_HAL_${_b})
        string(TOUPPER "${_b}" _B)
        if(_jr_has_${_b})
            set(JR_HAS_HAL_${_B} 1)
        else()
            set(JR_HAS_HAL_${_B} 0)
        endif()
    endif()
endforeach()

# ============================================================================
#  jr_core：**不依赖 rclcpp** 的实时核心
# ============================================================================
add_library(jr_core STATIC
    src/rt/jr_status.cpp
    src/rt/jr_param.cpp
    src/rt/jr_config.cpp
    src/rt/jr_bus_plan.cpp
    src/rt/jr_abi_check.cpp
    src/rt/jr_rt_sched.cpp
    src/rt/jr_bus_lock.cpp
    src/rt/jr_desc_cache.cpp
    src/rt/jr_bus_runtime.cpp
    src/rt/jr_bus_ops.cpp
    src/rt/jr_identify.cpp
    src/rt/jr_tick_group.cpp
)

target_include_directories(jr_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(jr_core PUBLIC ${JR_SDK_TARGET} Threads::Threads)

target_compile_definitions(jr_core PUBLIC
    JR_MAX_JOINTS_PER_BUS=${JR_MAX_JOINTS_PER_BUS}
    JR_HAS_HAL_VIRTUAL=${JR_HAS_HAL_VIRTUAL}
    JR_HAS_HAL_SLCAN=${JR_HAS_HAL_SLCAN}
    JR_HAS_HAL_SOCKETCAN=${JR_HAS_HAL_SOCKETCAN}
    JR_HAS_HAL_PCAN=${JR_HAS_HAL_PCAN}
)

# 警告：与 SDK 同风格（gcc/clang 一套，MSVC 一套）。
if(MSVC)
    target_compile_options(jr_core PRIVATE /W4 /utf-8)
    if(JR_WERROR)
        target_compile_options(jr_core PRIVATE /WX)
    endif()
else()
    target_compile_options(jr_core PRIVATE -Wall -Wextra -Wpedantic -Wshadow -Wconversion)
    if(JR_WERROR)
        target_compile_options(jr_core PRIVATE -Werror)
    endif()
endif()

# ============================================================================
#  可选目标：YAML 配置加载器（需要 yaml-cpp）
# ----------------------------------------------------------------------------
#  为什么不塞进 `jr_core`：核心库要能"只有编译器 + CMake"就编/测（Windows 开发机、
#  客户的交叉编译环境）；YAML 解析的依赖不该传染给它。
#  给谁用：WP3 的 ros2_control 组件（配置只能从 URDF 参数里拿路径）、
#  WP2 的节点、WP4 的工具 —— 三处用**同一份**配置文件格式（DESIGN §6.4）。
# ============================================================================
find_package(yaml-cpp QUIET)
if(yaml-cpp_FOUND)
    if(TARGET yaml-cpp::yaml-cpp)
        set(_jr_yaml_cpp yaml-cpp::yaml-cpp)   # yaml-cpp >= 0.8（Ubuntu 24.04）
    else()
        set(_jr_yaml_cpp yaml-cpp)             # yaml-cpp 0.7（Ubuntu 22.04）
    endif()
    add_library(jr_config_yaml STATIC src/config/jr_config_yaml.cpp)
    target_link_libraries(jr_config_yaml PUBLIC jr_core ${_jr_yaml_cpp})
    target_include_directories(jr_config_yaml PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
    if(MSVC)
        target_compile_options(jr_config_yaml PRIVATE /W4 /utf-8)
    else()
        target_compile_options(jr_config_yaml PRIVATE -Wall -Wextra -Wpedantic -Wshadow)
        if(JR_WERROR)
            target_compile_options(jr_config_yaml PRIVATE -Werror)
        endif()
    endif()
else()
    message(STATUS "jr_ros2: yaml-cpp not found -> YAML config loader (jr_config_yaml) skipped")
endif()

# ============================================================================
#  WP4 工具：**非 ROS**（不链 rclcpp）
# ----------------------------------------------------------------------------
#  为什么不做成 ROS 节点/`ros2 run`：这两个工具是**上机前/配置期**用的
#  （那时节点通常还没起来，甚至根本不用 ROS——客户的产线是裸 CMake）。
#  副作用是它们在这条"无 ROS 路径"里也能编、能测（`docker/run.sh` 阶段②、
#  Windows 开发机），于是 `jr_hw_verify --config <虚拟总线配置>` 天然就能进 CI。
#  ⚠ 与运行中的节点互斥：直连总线要拿单 master 锁（这正是设计要的效果）。
# ============================================================================
if(yaml-cpp_FOUND)
    foreach(_tool jr_hw_verify jr_gen_config jr_bus_plan)
        add_executable(${_tool} src/tools/${_tool}_main.cpp)
        target_link_libraries(${_tool} PRIVATE jr_core jr_config_yaml)
        target_include_directories(${_tool} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/rt)
        if(MSVC)
            target_compile_options(${_tool} PRIVATE /W4 /utf-8)
        else()
            target_compile_options(${_tool} PRIVATE -Wall -Wextra -Wpedantic)
            if(JR_WERROR)
                target_compile_options(${_tool} PRIVATE -Werror)
            endif()
        endif()
        install(TARGETS ${_tool} RUNTIME DESTINATION lib/${PROJECT_NAME})
    endforeach()
endif()

# ============================================================================
#  测试（纯 CMake 也能跑：colcon test / ctest 都走 CTest）
# ============================================================================
if(JR_BUILD_TESTS)
    # `BUILD_TESTING` 是 CTest 模块的开关：它为 OFF 时不会生成 DartConfiguration.tcl，
    # 于是 colcon test / ctest 每次都会往 stderr 打
    # "Cannot find file: .../DartConfiguration.tcl"（无害但吓人）。
    # ament 默认可能把它置 OFF，所以这里显式置 ON（仅本分支生效）。
    set(BUILD_TESTING ON)
    enable_testing()
    include(CTest)

    set(JR_TESTS
        test_triple_buffer
        test_spsc_ring
        test_param
        test_bus_plan
        test_config
        test_abi_check
        test_rt_sched
        test_bus_runtime
        test_ops
        test_desc_cache
        # `jr_gen_config` 的“命令行 → 扫描用总线配置”映射。**故意不挂在 yaml-cpp 下面**：
        # 它只用 jr_core 的 `validate_config()`，因此**无 ROS 的 Windows 开发机也能跑** ——
        # 而它守的两个 bug（slcan 漏 serial_baud / --is-fd 没进总线配置）恰恰是
        # “只有真机才暴露、CI 的 virtual 路径覆盖不到”的那类（§13.3-47）。
        test_gen_config_cfg
    )
    if(yaml-cpp_FOUND)
        list(APPEND JR_TESTS test_config_yaml)
    endif()

    foreach(_t ${JR_TESTS})
        add_executable(${_t} test/${_t}.cpp)
        target_link_libraries(${_t} PRIVATE jr_core)
        if(_t STREQUAL "test_config_yaml")
            target_link_libraries(${_t} PRIVATE jr_config_yaml)
        endif()
        target_include_directories(${_t} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
        if(MSVC)
            target_compile_options(${_t} PRIVATE /W4 /utf-8)
        else()
            target_compile_options(${_t} PRIVATE -Wall -Wextra)
        endif()
        add_test(NAME ${_t} COMMAND ${_t})
        # 集成测试需要真实时间（虚拟总线 + tick 线程），给足超时。
        set_tests_properties(${_t} PROPERTIES TIMEOUT 120)
    endforeach()

    # ---- WP4 工具的端到端测试（真跑 CLI；虚拟总线，无需硬件） ----
    #  ⚠ 只在工具存在时注册：没有 yaml-cpp 就没有 jr_config_yaml，也没有这两个工具
    #  （Windows 开发机就是这种情况）→ 本地测试数不变，容器里多一个 `tools_virtual`。
    if(TARGET jr_hw_verify AND TARGET jr_gen_config)
        add_test(NAME tools_virtual
                 COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/test/test_tools.sh")
        set_tests_properties(tools_virtual PROPERTIES
            ENVIRONMENT "JR_TOOLS_DIR=$<TARGET_FILE_DIR:jr_hw_verify>"
            TIMEOUT 180)
    endif()

    # ---- ROS 侧测试不在这里建 ----
    #  ⚠ 它们要链接 `jr_node`，而节点层在本文件的**后面**才创建
    #  （当时 `TARGET jr_node` 还不存在 → 整个块被静默跳过，测试数不会变）。
    #  见下面的 `if(JR_BUILD_TESTS AND TARGET jr_node)` 块。
endif()

# ============================================================================
#  节点层（WP2）：`jr_node` 目标 + `jr_bus` 可执行
# ----------------------------------------------------------------------------
#  只有 ament 模式 + 有 yaml-cpp 时才建：
#    - 节点必须读**共享的一份** YAML（§6.4），没有 yaml-cpp 就没法配置；
#    - Windows 上的纯 CMake 开发构建不装 ROS → 核心库照样能编能测（那才是它的价值）。
# ============================================================================
if(JR_WITH_ROS AND yaml-cpp_FOUND)
    find_package(rclcpp REQUIRED)
    find_package(rclcpp_lifecycle REQUIRED)
    find_package(diagnostic_updater REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(jr_interfaces REQUIRED)

    add_library(jr_node STATIC
        src/ros/jr_bus_node.cpp
        src/ros/jr_bus_services.cpp)

    target_include_directories(jr_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

    target_link_libraries(jr_node PUBLIC
        jr_core
        jr_config_yaml
        rclcpp::rclcpp
        rclcpp_lifecycle::rclcpp_lifecycle
        diagnostic_updater::diagnostic_updater
        ${std_msgs_TARGETS}
        ${sensor_msgs_TARGETS}
        # ⚠ 必须用带命名空间的名字（ament 给目标加了 `<pkg>::` 前缀，见 §13.3-14）
        jr_interfaces::jr_interfaces)

    if(MSVC)
        target_compile_options(jr_node PRIVATE /W4 /utf-8)
        if(JR_WERROR)
            target_compile_options(jr_node PRIVATE /WX)
        endif()
    else()
        target_compile_options(jr_node PRIVATE -Wall -Wextra -Wpedantic -Wshadow -Wconversion)
        if(JR_WERROR)
            target_compile_options(jr_node PRIVATE -Werror)
        endif()
    endif()

    add_executable(jr_bus src/ros/jr_bus_main.cpp)
    target_link_libraries(jr_bus PRIVATE jr_node)

    # ---- WP4 工具（**ROS 侧**）：`jr_ctl` 走服务，**一个 CAN 帧都不发** --------
    #  为什么它是 ROS 侧而是非 ROS：它调的是运行中节点的 19 个服务（§6.2），
    #  所以它是"节点跑着的时候**唯一**该用的运维入口"（DESIGN §12 首屏那条纪律）。
    add_executable(jr_ctl src/tools/jr_ctl_main.cpp)
    # ⚠ 必须用带命名空间的目标名（`rclcpp::rclcpp`）：裸名 `rclcpp` 会**链接不上 include 路径**，
    #   症状就是 `fatal error: rclcpp/rclcpp.hpp: No such file or directory`（实测踩过）。
    target_link_libraries(jr_ctl PRIVATE rclcpp::rclcpp jr_interfaces::jr_interfaces)
    if(MSVC)
        target_compile_options(jr_ctl PRIVATE /W4 /utf-8)
    else()
        target_compile_options(jr_ctl PRIVATE -Wall -Wextra -Wpedantic -Wshadow)
        if(JR_WERROR)
            target_compile_options(jr_ctl PRIVATE -Werror)
        endif()
    endif()
    install(TARGETS jr_ctl RUNTIME DESTINATION lib/${PROJECT_NAME})

    message(STATUS "jr_ros2: node layer enabled (target jr_node + executable jr_bus + tool jr_ctl)")
else()
    message(STATUS "jr_ros2: node layer skipped (needs ament + yaml-cpp)")
endif()

# ---- ROS 侧测试（必须放在节点层**之后**：要链接 jr_node）------------------
#  需要 rclcpp + 真 DDS（命令链路要验 QoS/命名空间/回调组），
#  所以**不在** Windows 的纯 CMake 构建里跑（那里没有 ROS，也不该假装有）。
if(JR_BUILD_TESTS AND TARGET jr_node)
    # 节点测试要订阅 `/diagnostics` 验证 §6.5（只有测试用，所以是 test_depend）
    find_package(diagnostic_msgs REQUIRED)

    # ---- WP4 工具（ROS 侧）：`jr_ctl` 的端到端用例（真起一个 `jr_bus` 节点）----
    #  ⚠ 这里要给**安装目录**而不是构建目录：脚本自己 source setup.bash。
    if(TARGET jr_ctl)
        #  ⚠ 工作区 setup.bash 只能由 CMake 自己算：colcon 用 --install-base 时
        #    `CMAKE_INSTALL_PREFIX` = `<install>/jr_ros2` ⇒ 上一级的 setup.bash 就是它。
        #    别在脚本里从二进制位置往上推：测试跑在 **build** 树，而 install 是它的**兄弟**目录
        #    （实测踩过：推出来变成 /tmp/setup.bash）。
        get_filename_component(_jr_install_root "${CMAKE_INSTALL_PREFIX}/.." ABSOLUTE)
        add_test(NAME jr_ctl_services
                 COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/test/test_jr_ctl.sh")
        set_tests_properties(jr_ctl_services PROPERTIES
            ENVIRONMENT "JR_BIN_DIR=$<TARGET_FILE_DIR:jr_ctl>;JR_WS_SETUP=${_jr_install_root}/setup.bash"
            TIMEOUT 300)
    endif()

    foreach(_t test_jr_bus_node)
        add_executable(${_t} test/${_t}.cpp)
        target_link_libraries(${_t} PRIVATE jr_node diagnostic_msgs::diagnostic_msgs)
        target_include_directories(${_t} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
        if(MSVC)
            target_compile_options(${_t} PRIVATE /W4 /utf-8)
        else()
            target_compile_options(${_t} PRIVATE -Wall -Wextra)
        endif()
        add_test(NAME ${_t} COMMAND ${_t})
        set_tests_properties(${_t} PROPERTIES TIMEOUT 120)
    endforeach()

    # §10.2 ⑦ 双进程抢总线：总线锁是**进程间** flock，只有在真的两个进程里才测得到
    # （同进程两个节点永远测不出"第二个进程抢不到锁"）。probe 只做 configure，
    # 退出码 0=成功 / 3=失败；脚本按退出码与日志判决，不猜。
    add_executable(jr_lock_probe test/jr_lock_probe.cpp)
    target_link_libraries(jr_lock_probe PRIVATE jr_node)
    if(MSVC)
        target_compile_options(jr_lock_probe PRIVATE /W4 /utf-8)
    else()
        target_compile_options(jr_lock_probe PRIVATE -Wall -Wextra)
    endif()
    add_test(NAME double_start
             COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/test/test_double_start.sh
                     $<TARGET_FILE:jr_lock_probe>)
    set_tests_properties(double_start PROPERTIES TIMEOUT 180)
endif()

# ============================================================================
#  安装（ROS 侧目标后续再加；核心库先给出来，方便其它包/工具链接）
# ============================================================================
# ⚠ ament 模式下必须把目标安装进 `jr_core` 这个 export set，否则 `ament_export_targets`
#   声明的导出集没人填 → CMake 生成阶段直接报
#   `INSTALL(EXPORT) given unknown export "jr_core"`（在 ROS 基础镜像里必现，
#   因为它的 CMAKE_PREFIX_PATH 预置了 /opt/ros，纯 CMake 也会走到 ament 分支）。
if(JR_WITH_ROS)
    set(_jr_core_export EXPORT jr_core)
    set(_jr_cfg_export  EXPORT jr_config_yaml)
else()
    set(_jr_core_export "")
    set(_jr_cfg_export  "")
endif()

install(TARGETS jr_core ${_jr_core_export}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(yaml-cpp_FOUND)
    # 与 jr_core 分开导出：消费者按需链接（不想要 YAML 的客户不引入 yaml-cpp）。
    install(TARGETS jr_config_yaml ${_jr_cfg_export}
            ARCHIVE DESTINATION lib
            LIBRARY DESTINATION lib
            RUNTIME DESTINATION bin)
endif()

if(TARGET jr_node)
    # `jr_node` / `jr_bus` **不进导出集**：目前没有别的包链接它们
    # （WP4 的工具走服务，不需要链接节点库）。等真需要时再加 export set，
    # 并且记得遵守"导出集顺序 = 依赖顺序"（见下面 ament_export_targets 的注释）。
    install(TARGETS jr_node jr_bus
            ARCHIVE DESTINATION lib
            LIBRARY DESTINATION lib
            RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()

if(JR_WITH_ROS)
    # ⚠⚠ 顺序就是依赖顺序：`ament_export_targets` 按**调用顺序**写进
    #   `<pkg>_TARGETS`，消费方的 config 就按这个顺序 include 导出文件；
    #   如果先 include 的那个引用了后面才定义的目标，消费方 `find_package` 直接失败：
    #     "The following imported targets are referenced, but are missing: jr_ros2::jr_core"
    #   （真实踩过：先导出了 jr_config_yaml，而它 PUBLIC 依赖 jr_core。）
    ament_export_targets(jr_core HAS_LIBRARY_TARGET)
    if(yaml-cpp_FOUND)
        ament_export_targets(jr_config_yaml HAS_LIBRARY_TARGET)
    endif()

    ament_export_dependencies(Threads)
    # ⚠ 只有"已安装的 SDK"这种模式才能把它导出去：
    #   源码模式（-DJRSDK_SOURCE_DIR=...）下 SDK 目标在本构建树内部，无法导出 ——
    #   消费者必须自己 `jr_locate_jsdk()`（见 jr_ros2_control/CMakeLists.txt 的说明）。
    if(JR_SDK_MODE STREQUAL "find_package")
        ament_export_dependencies(jsdk_can)
    endif()
    if(yaml-cpp_FOUND)
        ament_export_dependencies(yaml-cpp)
    endif()

    # ⚠ ament 包**必须**在最后调用它：负责安装 package.xml、生成
    #   share/<pkg>/local_setup.* 等资源索引。少了它 → colcon 能编过但包不"存在"
    #   （`ros2 pkg list` 里看不到，下游 find_package 也找不到）。
    ament_package()
endif()
