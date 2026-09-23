#!/usr/bin/env bash
# ============================================================================
#  docker/run.sh —— 在 **WSL 里**执行：建镜像（首次）+ 跑构建测试
# ----------------------------------------------------------------------------
#  用法：
#     cd /mnt/d/projects/cheetah/JointROS
#     bash docker/run.sh                  # 默认 jazzy
#     bash docker/run.sh humble           # 兼容性矩阵里的老发行版
#     bash docker/run.sh jazzy --no-cache # 强制重建镜像
#     JR_IMAGE_PREFIX=<镜像源> bash docker/run.sh jazzy
#
#  说明：
#   - 用 **只读** 挂载把父目录（cheetah/）挂进容器，容器内拷到 /tmp 再编：
#     Windows 侧的源码树不会被写入，Linux 产物也不会和 MinGW 的 build/ 混在一起；
#   - 所有构建都在容器内，宿主只需有 Docker 守护进程。
# ============================================================================
set -euo pipefail

DISTRO="${1:-jazzy}"
shift || true
NO_CACHE=""
for a in "$@"; do
    [[ "$a" == "--no-cache" ]] && NO_CACHE="--no-cache"
done

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
PARENT="$(cd "$REPO/.." && pwd)"          # 含 JointROS 与 JointSDK

if [[ ! -d "$PARENT/JointSDK" ]]; then
    echo "error: $PARENT/JointSDK 不存在（容器需要 JointSDK 源码）" >&2
    exit 2
fi

# 宿主目录 → 容器路径（WSL 下 /mnt/d/... 可直接挂）
MOUNT_HOST="$PARENT"
IMAGE="jr-ros2:${DISTRO}"
BASE_IMAGE="${JR_IMAGE_PREFIX:+${JR_IMAGE_PREFIX%/}/}ros:${DISTRO}-ros-base"

DOCKER="docker"
if ! docker info >/dev/null 2>&1; then
    if sudo -n docker info >/dev/null 2>&1; then
        DOCKER="sudo docker"
        echo "[warn] 当前用户不在 docker 组，改用 sudo（建议执行：sudo usermod -aG docker \$USER 后重开 WSL）"
    else
        cat >&2 <<'EOF'
error: 连不上 Docker 守护进程。请在 WSL 里执行（需要你输入密码）：

    sudo service docker start

若希望以后每次开机自动启动（且不需要 sudo），可执行：

    sudo usermod -aG docker $USER
    printf '[boot]\ncommand = service docker start\n' | sudo tee /etc/wsl.conf

然后在 **Windows** 侧执行 `wsl --shutdown` 重进 WSL。
EOF
        exit 3
    fi
fi

echo "== 构建镜像 $IMAGE（基础镜像 $BASE_IMAGE）=="
$DOCKER build $NO_CACHE -t "$IMAGE" --build-arg "BASE_IMAGE=$BASE_IMAGE" "$HERE"

echo "== 在容器内构建 + 测试（发行版 $DISTRO）=="
$DOCKER run --rm \
    -v "$MOUNT_HOST":/host:ro \
    -e JR_SRC_RO=/host/JointROS \
    -e JR_SDK_RO=/host/JointSDK \
    -e JR_CMAKE_ARGS="${JR_CMAKE_ARGS:-}" \
    "$IMAGE" \
    bash /host/JointROS/docker/container_build.sh
