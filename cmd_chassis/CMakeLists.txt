cmake_minimum_required(VERSION 3.11)
project(cmd_chassis)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


set(ros_dependencies
  rclcpp
  rm_interfaces
  nav_msgs
  nav2_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
)

add_executable(fake_joint src/fake_joint.cpp)
ament_target_dependencies(fake_joint ${ros_dependencies})

add_executable(rot_imu src/rot_imu.cpp)
ament_target_dependencies(rot_imu ${ros_dependencies})

add_executable(twist_transformer src/twist_transformer.cpp)
ament_target_dependencies(twist_transformer ${ros_dependencies})

add_executable(twist2chassis_cmd src/twist2chassis_cmd.cpp)
ament_target_dependencies(twist2chassis_cmd ${ros_dependencies})

# install(
#   DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

# Install nodes
install(
  TARGETS fake_joint rot_imu twist_transformer twist2chassis_cmd
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
