cmake_minimum_required(VERSION 3.8)
project(mas_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MineBelief.msg"
  "msg/PoseBeacon.msg"
  "msg/MineDelta.msg"
  "msg/SyncHello.msg"
  "msg/SyncAck.msg"
  "msg/TaskAnnounce.msg"
  "msg/TaskClaim.msg"
  "msg/TaskResult.msg"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
