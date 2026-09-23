#!/usr/bin/env bash
# ============================================================================
#  tools/build_dev.sh —— 一条命令：配置 + 构建 + 跑测试（**不需要 ROS**）
# ----------------------------------------------------------------------------
#  为什么要这个脚本：核心库 `jr_core`（无 rclcpp 依赖）必须能在"只有编译器 +
#  CMake"的机器上验证，包括虚拟总线端到端集成测试。装了 ROS 的机器请用 colcon
#  （见 README §构建）。
#
#  用法：
#     tools/build_dev.sh                       # 用 ../JointSDK 源码构建
#     JRSDK_SOURCE_DIR=/path/to/JointSDK tools/build_dev.sh
#     tools/build_dev.sh -DJR_WERROR=ON        # 额外 CMake 参数会透传
# ============================================================================
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SDK="${JRSDK_SOURCE_DIR:-$(cd "$ROOT/.." && pwd)/JointSDK}"
BUILD="${JR_BUILD_DIR:-$ROOT/build}"

if [[ ! -f "$SDK/CMakeLists.txt" ]]; then
    echo "error: JointSDK not found at '$SDK'." >&2
    echo "       set JRSDK_SOURCE_DIR, or install it so find_package(jsdk_can) works." >&2
    exit 2
fi

# 生成器：Windows（MinGW）必须显式指定，否则 CMake 会挑到 VS/Ninja 而失败。
if [[ -z "${JR_GENERATOR:-}" ]]; then
    case "$(uname -s)" in
        MINGW*|MSYS*|CYGWIN*) JR_GENERATOR="MinGW Makefiles" ;;
        *)                    JR_GENERATOR="Unix Makefiles" ;;
    esac
fi

# 构建类型默认 RelWithDebInfo（= -O2）**不是随便定的**：GCC 的 -Wformat-truncation 等
# 一批告警只在优化下才启用 —— 曾经因为本机开发构建不带 -O，漏掉了两个真实的 snprintf
# 截断风险，直到进了 Linux 容器（-O2 + -Werror）才炸出来。
echo "== configure ($JR_GENERATOR, ${JR_BUILD_TYPE:-RelWithDebInfo}) =="
cmake -S "$ROOT" -B "$BUILD" -G "$JR_GENERATOR" -DJRSDK_SOURCE_DIR="$SDK" \
      -DCMAKE_BUILD_TYPE="${JR_BUILD_TYPE:-RelWithDebInfo}" "$@"

echo "== build =="
cmake --build "$BUILD" -j "${JR_JOBS:-4}"

echo "== test =="
# CMake ≥3.20 才支持 --test-dir；老版本回退到 cd（与 SDK 的 wsl_build.sh 一致）。
if ctest --test-dir "$BUILD" --output-on-failure; then
    :
else
    ( cd "$BUILD" && ctest --output-on-failure )
fi
