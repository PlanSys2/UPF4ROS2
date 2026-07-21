project(plansys2_upf_plan_solver)

cmake_minimum_required(VERSION 3.5)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(plansys2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(upf_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
  rclcpp
  rclcpp_action
  ament_index_cpp
  plansys2_core
  pluginlib
  upf_msgs
)

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/plansys2_upf_plan_solver/upf_plan_solver.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(plansys2_core plansys2_upf_plan_solver_plugin.xml)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES plansys2_upf_plan_solver_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)



install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake)
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()