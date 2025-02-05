project(upf_msgs)

cmake_minimum_required(VERSION 3.5)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AbstractTaskDeclaration.msg"
  "msg/Effect.msg"
  "msg/Variable.msg"
  "msg/Task.msg"
  "msg/PlanGenerationResult.msg"
  "msg/Method.msg"
  "msg/ObjectDeclaration.msg"
  "msg/ActionInstance.msg"
  "msg/Interval.msg"
  "msg/TimedEffect.msg"
  "msg/Duration.msg"
  "msg/Real.msg"
  "msg/PDDLPlanRequest.msg"
  "msg/TypeDeclaration.msg"
  "msg/Action.msg"
  "msg/PlanRequest.msg"
  "msg/PlanRequestRemote.msg"
  "msg/Hierarchy.msg"
  "msg/LogMessage.msg"
  "msg/ExpressionItem.msg"
  "msg/ValidationResult.msg"
  "msg/CompilerResult.msg"
  "msg/Condition.msg"
  "msg/Expression.msg"
  "msg/GoalWithCost.msg"
  "msg/Plan.msg"
  "msg/Assignment.msg"
  "msg/Fluent.msg"
  "msg/Problem.msg"
  "msg/Goal.msg"
  "msg/Atom.msg"
  "msg/Schedule.msg"
  "msg/Timepoint.msg"
  "msg/EffectExpression.msg"
  "msg/ValidationRequest.msg"
  "msg/TaskNetwork.msg"
  "msg/TimeInterval.msg"
  "msg/Timing.msg"
  "msg/Parameter.msg"
  "msg/Metric.msg"
  "msg/ValidationMetric.msg"
  "srv/GetProblem.srv"
  "srv/NewProblem.srv"
  "srv/SetProblem.srv"
  "srv/AddFluent.srv"
  "srv/AddAction.srv"
  "srv/AddObject.srv"
  "srv/SetInitialValue.srv"
  "srv/AddGoal.srv"
  "srv/PDDLPlanOneShot.srv"
  "srv/PlanOneShot.srv"
  "action/PDDLPlanOneShot.action"
  "action/PlanOneShot.action"
  "action/PlanOneShotRemote.action"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
