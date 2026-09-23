# 容器化构建与测试（WSL2 + Docker）

> 为什么用容器：本机（Windows + MinGW）**编不了也测不了** ROS 侧代码，而且
> `jr_core` 里有一批**只在 Linux 上编译**的路径（`flock`、`SCHED_FIFO`、`mlockall`、
> `clock_nanosleep`、SocketCAN 头文件）。在容器里一次把两者都覆盖，并且能跑
> Humble / Jazzy / Lyrical 的**兼容性矩阵**。

## 一次性准备（需要你手动执行，因为要 sudo）

在 **WSL** 里：

```bash
# 1) 启动 Docker 守护进程
sudo service docker start

# 2) 让你自己不用 sudo 也能用 docker（推荐；之后重开 WSL 生效）
sudo usermod -aG docker $USER

# 3) 让守护进程开机自启（可选，但强烈建议：否则每次重开 WSL 都要再 start 一次）
printf '[boot]\ncommand = service docker start\n' | sudo tee /etc/wsl.conf
```

做完 2)、3) 后，在 **Windows** 侧执行 `wsl --shutdown`，再重新进入 WSL。

## 每次构建测试（一条命令）

在 **WSL** 里：

```bash
cd /mnt/d/projects/cheetah/JointROS
bash docker/run.sh            # 默认 Jazzy
bash docker/run.sh humble     # 换发行版
bash docker/run.sh lyrical    # 主目标发行版
bash docker/run.sh jazzy --no-cache
```

它会：
1. 建镜像 `jr-ros2:<distro>`（首次约几分钟；镜像不 COPY 源码，可复用）；
   顺带说明：镜像里先 `apt-get upgrade`，让**基础镜像与 apt 仓库同一批** ——
   否则会撞上 ABI 错批（实测 Lyrical：`ros2_control_node` 一启动就 `undefined symbol`，
   而构建全绿）。代价是冷构建慢一些，进层缓存后无感。
2. 以**只读**方式挂载 `cheetah/` 到容器 `/host`，容器内拷到 `/tmp` 再编
   —— Windows 侧源码树不会被写入，Linux 产物也不会和 MinGW 的 `build/` 混在一起；
3. 跑**三条路径**（分别验证三种构建模式）：
   - **[2/4] 无 ROS 路径**：`cmake` + `ctest`，带 `-DCMAKE_DISABLE_FIND_PACKAGE_ament_cmake=ON`
     —— 这是客户非 ROS 集成/交叉编译场景的真实写照（若不关，ROS 基础镜像的 `CMAKE_PREFIX_PATH`
     会让 `find_package(ament_cmake)` 命中，这条路径就名存实亡）；
     它也顺便跑 **WP4 工具**的端到端用例（`tools_virtual`：`jr_hw_verify` + `jr_gen_config`，
     虚拟总线、不需要硬件）；
   - **[3/4] ament/colcon 路径**：`colcon build --base-paths jr_ros2 jr_ros2_control` + `colcon test`；
   - **[4/4] JTC 端到端**：起 `controller_manager` + `JointTrajectoryController` + 虚拟总线，
     发一条真轨迹并断言终点误差；**`mit` 与 `csp` 两种模式都跑**（命令接口集不同 ⇒ demo 的
     URDF 与控制器 claim 都按模式生成，WP3 验收，详见 `jr_ros2_control/test/jtc_demo/`）。

## 已验证结果

| 发行版 | 基础镜像 | `ctest`（无 ROS 路径） | `colcon test`（ament 路径） | JTC 端到端（mit / csp） |
|---|---|---|---|---|
| Jazzy | Ubuntu 24.04 / gcc 13.3 / CMake 3.28 | **12/12 通过**（含 `tools_virtual`） | **20 tests, 0 errors, 0 failures**（jr_ros2 + jr_ros2_control） | **都 PASS**（mit 0.0080 rad / csp 0.00035 rad） |
| Humble | Ubuntu 22.04 / gcc 11.4 / CMake 3.22 | **12/12 通过** | **20 tests, 0 errors, 0 failures** | **都 PASS**（mit 0.0080 / csp 0.00035） |
| Lyrical（主目标） | Ubuntu 26.04 / gcc 15.2 / CMake 4.2 | **12/12 通过** | **20 tests, 0 errors, 0 failures** | **都 PASS**（mit 0.0080 / csp 0.00035） |

## 两条容易踩的坑（脚本已处理，但值得知道）

1. **不要在仓库根目录直接 `colcon build`**：根目录的 `CMakeLists.txt`（纯 CMake 开发入口）会被
   colcon 识别成一个 `cmake` 包（`colcon list` 显示 `jointros_dev  .`）并且**不再深入子目录** ——
   ament 包 `jr_ros2` 根本没被构建（`ament_package()` 没跑），但 10 个测试却照样通过，很容易误判为成功。
   所以脚本用 `--base-paths jr_ros2 jr_ros2_control`。
2. `set -u` 与 ROS 的 `setup.bash` 不兼容（它引用未定义变量）→ 脚本在 `source` 前后临时 `set +u`。
3. **基础镜像与 apt 仓库错批**：镜像里的核心库可能比仓库旧一批，`apt install` 后运行期报
   `undefined symbol`（构建却是绿的）。判据：
   `nm -D /opt/ros/$ROS_DISTRO/lib/libservice_msgs*typesupport_fastrtps_c.so | grep -c has_buffer_fields`
   （0 即中招）。镜像里已用 `apt-get upgrade -y` 对齐。

## 网络：拉不动 docker.io 时

`docker.io` 在国内经常拉不动。指定镜像源即可（`JR_IMAGE_PREFIX` 会拼在 `ros:` 前面）：

```bash
JR_IMAGE_PREFIX=<你的镜像源> bash docker/run.sh jazzy
# 例如：JR_IMAGE_PREFIX=docker.m.daocloud.io bash docker/run.sh jazzy
```

已验证可用的镜像源（本机实测）：`docker.m.daocloud.io`、`docker.1ms.run`、`hub.rat.dev`（
三个都能拉到 `ros:lyrical-ros-base`）；`docker.1panel.live` 对 `ros:lyrical-ros-base` 返回 **denied**；
`dockerpull.org` 不通。另外：**容器内的 apt 是通的**（`apt-get update` 约 7 s），无需换 apt 源。
若镜像源全不通，告诉我报错原文，我改用「`ubuntu:` 基础镜像 + 国内 ROS 2 apt 源」的 Dockerfile
（不依赖 Docker Hub 的 ROS 镜像，只依赖 apt 源）。

## 真机（CAN 适配器）说明

容器里目前做的是**构建 + 虚拟总线测试**（不需要硬件）。要把 USB-CAN 适配器透进 WSL2
需要 `usbipd-win`（Windows 侧装 + 以管理员绑定设备），再在容器里 `--device`/`--net=host`；
真机联调建议直接在 WSL 里原生跑（不经容器），细节到 WP4「真机分层自检」时再定。
