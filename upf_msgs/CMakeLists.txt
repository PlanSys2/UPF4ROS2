project(upf_msgs)

cmake_minimum_required(VERSION 3.5)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DurativeAction.msg"
  "msg/InstantaneousAction.msg"
  "msg/Plan.msg"
  "msg/TimedAction.msg"
  "msg/UserType.msg"
  "msg/Fluent.msg"
  "msg/Parameter.msg"
  "msg/Problem.msg"
  "msg/TimedFluent.msg" 
  "srv/AddAction.srv"
  "srv/AddFluent.srv"
  "srv/AddObject.srv"
  "srv/AddProblem.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
