cmake_minimum_required(VERSION 3.11)
project(rm_base)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Default Release
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# -------- Find thirdparty library ------- #
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rmoss_base REQUIRED)
find_package(rmoss_interfaces REQUIRED)
find_package(rm_interfaces REQUIRED)

set(dependencies
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2_ros
  rmoss_base
  rmoss_interfaces
  rm_interfaces
)

# ------------- Build library ------------ #

# file(GLOB_RECURSE buffer_processors CONFIGURE_DEPENDS "src/buffer_processor/*.cpp")

add_library(${PROJECT_NAME} SHARED
  "src/robot_base_node.cpp"
  # "src/buffer_processor_factory.cpp"
  # ${buffer_processors}
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "rm_base::RobotBaseNode"
  EXECUTABLE robot_base_node
)

# ------------ Build execuable ----------- #

# ---------------- Install --------------- #

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install  directories
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ----------------- Test ----------------- #
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
#   add_subdirectory(test)
# endif()

ament_package()
