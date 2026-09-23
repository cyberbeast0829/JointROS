# ============================================================================
#  jr_interfaces —— 消息与服务定义
# ----------------------------------------------------------------------------
#  刻意保持"零业务依赖"：只依赖 std_msgs / builtin_interfaces。
#  这样客户自研的控制器（人形 WBC、机器狗步态）只装这一个包就能对接，
#  不必被拖进 ros2_control / 驱动库 / SDK。
#
#  字段语义与单位见各 .msg / .srv 里的注释（DESIGN §6.1/§6.2 是契约正本）。
# ============================================================================
cmake_minimum_required(VERSION 3.16)
project(jr_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    # ── 反馈与状态 ────────────────────────────────────────────────
    "msg/JointFeedback.msg"
    "msg/JointFeedbackArray.msg"
    "msg/BusStatus.msg"
    "msg/RtStats.msg"
    "msg/JointFault.msg"
    # ── 命令 ──────────────────────────────────────────────────────
    "msg/MitCommand.msg"
    "msg/MitCommandArray.msg"
    "msg/JointCommand.msg"
    "msg/JointCommandArray.msg"
    # ── 运维/参数的支撑类型 ───────────────────────────────────────
    "msg/JointResult.msg"
    "msg/ParamValue.msg"
    "msg/ParamWrite.msg"
    "msg/ParamWriteResult.msg"
    "msg/EndpointInfo.msg"
    "msg/DescriptorInfo.msg"
    "msg/DeviceInfo.msg"
    # ── 服务（§6.2）：运动/生命周期 ───────────────────────────────
    "srv/SetEnabled.srv"
    "srv/Calibrate.srv"
    "srv/Home.srv"
    "srv/SetZero.srv"
    "srv/SaveConfig.srv"
    "srv/ResetDevice.srv"
    "srv/SetNodeId.srv"
    "srv/FaultReset.srv"
    "srv/Jog.srv"
    # ── 服务：参数/描述符/诊断/产线 ───────────────────────────────
    "srv/ReadParams.srv"
    "srv/WriteParams.srv"
    "srv/ListEndpoints.srv"
    "srv/LookupEndpoint.srv"
    "srv/GetDeviceInfo.srv"
    "srv/GetBusStats.srv"
    "srv/GetDescriptorInfo.srv"
    "srv/ExportDescriptor.srv"
    "srv/ImportDescriptor.srv"
    "srv/PublishHeartbeatHint.srv"
    DEPENDENCIES
        std_msgs
        builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
