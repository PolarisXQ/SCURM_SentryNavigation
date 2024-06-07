# Tests
ament_add_gtest(velocity_smoother_ext_tests
  test_velocity_smoother_ext.cpp
)
ament_target_dependencies(velocity_smoother_ext_tests
  ${dependencies}
)
target_link_libraries(velocity_smoother_ext_tests
  ${library_name}
)
