cmake_minimum_required(VERSION 3.5)
project(terrain_analysis_ext)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)

add_executable(terrainAnalysisExt src/terrainAnalysisExt.cpp)
ament_target_dependencies(terrainAnalysisExt rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs message_filters pcl_ros pcl_conversions)


add_executable(PCDPublisherNode src/PCDPublisherNode.cpp)
ament_target_dependencies(PCDPublisherNode rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs message_filters pcl_ros pcl_conversions)

add_executable(PCDSaverNode src/PCDSaverNode.cpp)
ament_target_dependencies(PCDSaverNode rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs message_filters pcl_ros pcl_conversions)

add_executable(pathNorm src/pathNorm.cpp)
ament_target_dependencies(pathNorm rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs message_filters pcl_ros pcl_conversions)
target_link_libraries(pathNorm ${PCL_LIBRARIES})

install(TARGETS
  terrainAnalysisExt
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  PCDPublisherNode
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  PCDSaverNode
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
pathNorm
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
  launch
  maps
  rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
