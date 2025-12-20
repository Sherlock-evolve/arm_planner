cmake_minimum_required(VERSION 3.22)
project(my_arm_config)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

set(action_files
  action/PlanExecutePose.action
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  DEPENDENCIES geometry_msgs
)

add_executable(moveit_motion_server
  src/moveit_motion_server.cpp
)
ament_target_dependencies(moveit_motion_server
  rclcpp
  rclcpp_action
  geometry_msgs
  moveit_ros_planning_interface
  tf2_geometry_msgs
  tf2_ros
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(moveit_motion_server ${cpp_typesupport_target})

add_executable(qt_arm_client
  src/qt_arm_app.cpp
)
target_link_libraries(qt_arm_client
  Qt5::Widgets
)
ament_target_dependencies(qt_arm_client
  rclcpp
  rclcpp_action
  geometry_msgs
  tf2_geometry_msgs
  tf2_ros
)
target_link_libraries(qt_arm_client ${cpp_typesupport_target})

install(TARGETS
  qt_arm_client
  moveit_motion_server
  DESTINATION lib/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(
  rclcpp
  rclcpp_action
  geometry_msgs
  moveit_ros_planning_interface
  tf2_geometry_msgs
  tf2_ros
  rosidl_default_runtime
)

ament_package()
