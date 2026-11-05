cmake_minimum_required(VERSION 3.8)
project(ss_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/LocatePanels.action"
  "msg/PanelTop.msg"
  "msg/CleanupDone.msg"
  "msg/LocatorStatus.msg"
  DEPENDENCIES std_msgs unique_identifier_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
