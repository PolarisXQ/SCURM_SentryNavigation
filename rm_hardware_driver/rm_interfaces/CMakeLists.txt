cmake_minimum_required(VERSION 3.8)
project(rm_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # "msg/referee/EventData.msg"
  # "msg/referee/GameResult.msg"
  # "msg/referee/GameRobotHp.msg"
  # "msg/referee/RobotPos.msg"
  # "msg/referee/RobotStatus.msg"
  # "msg/referee/GameStatus.msg"
  # "msg/referee/PowerHeatData.msg"
  # "msg/referee/RobotBuff.msg"
  # "msg/referee/HurtData.msg"
  # "msg/referee/RobotInteractiveData.msg"
  # "msg/referee/ShootData.msg"

  "msg/referee/GameState.msg"
  "msg/referee/SentryCmd.msg"
  "msg/ChassisCmd.msg"
  "msg/Gimbal.msg"
  "msg/GimbalCmd.msg"
  "msg/ShootCmd.msg"

DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
