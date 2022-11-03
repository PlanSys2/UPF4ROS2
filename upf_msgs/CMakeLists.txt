project(upf_msgs)

cmake_minimum_required(VERSION 3.5)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Expression.msg"
  "msg/Atom.msg"
  "msg/Real.msg"
  "msg/ActionInstance.msg"
  "msg/Plan.msg"
  "msg/PDDLPlanRequest.msg"
  "msg/LogMessage.msg"
  "msg/PlanGenerationResult.msg"
  "action/PDDLPlanOneShot.action"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
