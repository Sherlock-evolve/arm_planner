cmake_minimum_required(VERSION 3.8)
project(my_arm_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
# 确保 ros2_control_test_assets 存在如果您的URDF使用了它
find_package(ros2_control_test_assets REQUIRED) 

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ----------------------------------------------------
# 关键安装步骤：将文件复制到 install 目录
# ----------------------------------------------------

# 这一步确保将 'urdf' 文件夹下的所有文件复制到 ROS 2 的安装路径中
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()