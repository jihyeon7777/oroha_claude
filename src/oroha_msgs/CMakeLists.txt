cmake_minimum_required(VERSION 3.8)
project(oroha_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PowerSample.msg"
  "msg/ExperimentEvent.msg"
  "msg/ExperimentStatus.msg"
  "srv/ArmExperiment.srv"
  "srv/AddNote.srv"
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
