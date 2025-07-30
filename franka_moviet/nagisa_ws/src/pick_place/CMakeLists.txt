cmake_minimum_required(VERSION 3.8)
project(pick_place)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# 移除错误的部分 - Python 文件不应使用 add_executable
## 不要使用 add_executable 处理 Python 文件

# 正确安装 Python 脚本
install(
  PROGRAMS 
    src/task.py  # 确保这是正确的路径
  DESTINATION lib/${PROJECT_NAME}
)
# 安装启动文件
# 安装launch目录下的内容（不包含launch目录自身）
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 安装config目录下的内容（不包含config目录自身）
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()