# ============================================================================
#  jr_sdk.cmake —— 定位 JointSDK 并注入"容量必须一致"的编译宏
# ============================================================================
#  三种获取方式（DESIGN ADR-15），按优先级探测：
#    ① find_package(jsdk_can CONFIG)      已安装的 SDK（客户/产线，推荐）
#    ② JRSDK_SOURCE_DIR=<JointSDK 源码>    add_subdirectory（开发；可把容量宏调大）
#    ③ JRSDK_GIT_REPOSITORY=<url>          FetchContent（CI 拉固定版本）
#
#  ⚠ 容量一致性的硬要求
#   SDK 的 `JSDK_MAX_JOINTS_STATIC` / `JSDK_CONTEXT_MAX_SIZE` 是**编译期**常量，
#   而我们的 `JR_MAX_JOINTS_PER_BUS` 必须与前者相等。用 add_subdirectory 时我们
#   用 PUBLIC 编译宏把两者一起注入（SDK 与我们的 TU 看到同一组值）；
#   用已安装的 SDK 时无法注入，只能靠**启动自检**（jsdk_context_size/add_joint）
#   兜底 —— 因此 `jr_core` 绝不假设这两个常量，而是用运行时尺寸分配（ADR-12）。
# ============================================================================

include(CMakeParseArguments)

set(JR_MAX_JOINTS_PER_BUS 16 CACHE STRING
    "每个 jsdk_context 支持的关节数上限（会作为 JSDK_MAX_JOINTS_STATIC 注入 SDK）")
set(JR_CONTEXT_MAX_SIZE 12288 CACHE STRING
    "与 JR_MAX_JOINTS_PER_BUS 匹配的 JSDK_CONTEXT_MAX_SIZE（上下文存储字节数）")
set(JRSDK_SOURCE_DIR "" CACHE PATH "JointSDK 源码目录（方式②）")
set(JRSDK_GIT_REPOSITORY "" CACHE STRING "JointSDK git 仓库（方式③）")

function(jr_locate_jsdk)
    if(TARGET jsdk::can)
        set(JR_SDK_TARGET "jsdk::can" PARENT_SCOPE)
        set(JR_SDK_MODE "find_package" PARENT_SCOPE)
        message(STATUS "jr: using installed JointSDK target jsdk::can")
        return()
    endif()

    find_package(jsdk_can QUIET CONFIG)
    if(jsdk_can_FOUND)
        set(JR_SDK_TARGET "jsdk::can" PARENT_SCOPE)
        set(JR_SDK_MODE "find_package" PARENT_SCOPE)
        message(STATUS "jr: found installed JointSDK (jsdk_can ${jsdk_can_VERSION})")
        return()
    endif()

    if(JRSDK_SOURCE_DIR AND EXISTS "${JRSDK_SOURCE_DIR}/CMakeLists.txt")
        # 只编我们需要的部分：不要测试/CLI/示例/Python，避免把 SDK 的依赖拖进来。
        set(JSDK_BUILD_TESTS      OFF CACHE BOOL "" FORCE)
        set(JSDK_BUILD_EXAMPLES   OFF CACHE BOOL "" FORCE)
        set(JSDK_BUILD_CLI        OFF CACHE BOOL "" FORCE)
        set(JSDK_BUILD_PYTHON     OFF CACHE BOOL "" FORCE)
        set(JSDK_BUILD_SHARED     OFF CACHE BOOL "" FORCE)
        set(JSDK_ENABLE_HEAP      OFF CACHE BOOL "" FORCE)
        set(JSDK_BUILD_HAL_VIRTUAL ON CACHE BOOL "" FORCE)   # CI/演示必需
        set(JSDK_BUILD_HAL_SLCAN   ON CACHE BOOL "" FORCE)
        add_subdirectory("${JRSDK_SOURCE_DIR}" "${CMAKE_BINARY_DIR}/jsdk_can" EXCLUDE_FROM_ALL)

        # ⚠⚠ 必须补一个 `jsdk::can` 别名（真实踩过）：
        #   SDK 自己的安装导出写的是 `install(EXPORT jsdk_canTargets ... NAMESPACE jsdk::)`，
        #   所以**装完后**的消费者看到的目标名是 `jsdk::can`；而源码模式（add_subdirectory）
        #   下只有 `jsdk_can`。后果：`ament_export_targets(jr_core)` 会把 jr_core 的链接依赖
        #   按 SDK 的导出名记成 `jsdk::can` → 消费包 `find_package(jr_ros2)` 直接报
        #   "The following imported targets are referenced, but are missing: jsdk::can"。
        if(NOT TARGET jsdk::can)
            add_library(jsdk::can ALIAS jsdk_can)
        endif()

        set(JR_SDK_TARGET "jsdk_can" PARENT_SCOPE)
        set(JR_SDK_MODE "subdirectory" PARENT_SCOPE)

        # 容量宏：PUBLIC → SDK 与我们自己的 TU 都看到同一组值（这才是"真的可配"）。
        target_compile_definitions(jsdk_can PUBLIC
            JSDK_MAX_JOINTS_STATIC=${JR_MAX_JOINTS_PER_BUS}
            JSDK_CONTEXT_MAX_SIZE=${JR_CONTEXT_MAX_SIZE})
        message(STATUS "jr: using JointSDK from source: ${JRSDK_SOURCE_DIR} "
                       "(MAX_JOINTS_STATIC=${JR_MAX_JOINTS_PER_BUS})")
        return()
    endif()

    if(JRSDK_GIT_REPOSITORY)
        include(FetchContent)
        FetchContent_Declare(jsdk_can GIT_REPOSITORY "${JRSDK_GIT_REPOSITORY}" GIT_TAG "${JRSDK_GIT_TAG}")
        set(JSDK_BUILD_TESTS OFF CACHE BOOL "" FORCE)
        set(JSDK_BUILD_CLI OFF CACHE BOOL "" FORCE)
        FetchContent_MakeAvailable(jsdk_can)
        if(NOT TARGET jsdk::can)          # 同上：与 SDK 安装后的导出名保持一致
            add_library(jsdk::can ALIAS jsdk_can)
        endif()
        set(JR_SDK_TARGET "jsdk_can" PARENT_SCOPE)
        set(JR_SDK_MODE "fetchcontent" PARENT_SCOPE)
        target_compile_definitions(jsdk_can PUBLIC
            JSDK_MAX_JOINTS_STATIC=${JR_MAX_JOINTS_PER_BUS}
            JSDK_CONTEXT_MAX_SIZE=${JR_CONTEXT_MAX_SIZE})
        return()
    endif()

    message(FATAL_ERROR
        "JointSDK not found. Provide ONE of:\n"
        "  - an installed SDK so that find_package(jsdk_can) works (recommended), or\n"
        "  - -DJRSDK_SOURCE_DIR=/path/to/JointSDK (source integration), or\n"
        "  - -DJRSDK_GIT_REPOSITORY=<url> [-DJRSDK_GIT_TAG=<tag>] (CI).")
endfunction()
