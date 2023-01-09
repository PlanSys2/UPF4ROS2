ament_add_gtest(upf_test upf_test.cpp)
target_link_libraries(upf_test ${PROJECT_NAME} dl)
target_compile_definitions(upf_test PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
