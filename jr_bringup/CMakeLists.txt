# ============================================================================
#  jr_bringup —— 示例与启动文件（WP7）
# ----------------------------------------------------------------------------
#  本包**没有 C++ 代码**：只装 launch / config / urdf / docs，
#  再挂一个“示例必须真能跑起来”的 CI 冒烟用例（验收口径：`ros2 launch` 一条命令可复现）。
# ============================================================================
cmake_minimum_required(VERSION 3.16)
project(jr_bringup NONE)   # ⚠ CMake 4 起 `project()` 的 LANGUAGES 参数必填（可写 NONE）

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config urdf docs
        DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # 冒烟：真跑 `ros2 launch`，断言节点走到 active、`jr_ctl` 能调通服务。
  # ⚠ `jr_bus` 是**生命周期节点**：只 `ros2 run` 什么都不建（§13.3-36）⇒
  #   这个用例就是“示例不能只是文档承诺”的那道闸。
  #
  # ⚠ 这里**不能**用 `$<TARGET_FILE_DIR:jr_ros2>` 去找 `jr_bus`/`jr_ctl`：
  #   那是**别的包**里的目标，跳包引用不成立（构建期直接报错）。
  #   脚本改成走安装空间的 `ros2 run` / `ros2 launch`，只需要 JR_WS_SETUP。
  #
  # ⚠⚠ 必须显式写 `bash`，**不能**直接 `COMMAND <脚本路径>`：本仓库的文件在索引里
  #   统一是 `100644`（没带可执行位，见 `git ls-files -s`），直接在 Linux 上克隆后
  #   跑就是 "Permission denied" 而不是测试失败。开发机是 Windows（drvfs 挂载把
  #   所有文件都报成可执行），**这个坑在开发机上永远看不见** —— 既有测试
  #   （`test_tools.sh` / `test_jr_ctl.sh`）全都写成 `COMMAND bash …` 就是这个原因。
  get_filename_component(_jr_install_root "${CMAKE_INSTALL_PREFIX}/.." ABSOLUTE)
  add_test(NAME launch_smoke
           COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/test/test_launch_smoke.sh")
  set_tests_properties(launch_smoke PROPERTIES
    ENVIRONMENT "JR_WS_SETUP=${_jr_install_root}/setup.bash"
    TIMEOUT 900)
endif()

ament_package()
