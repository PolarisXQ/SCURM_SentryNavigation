cmake_minimum_required(VERSION 3.5)
project(icp_relocalization)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wextra -Wpedantic)
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g")
set(CMAKE_BUILD_TYPE Release)

set(USE_LIVOX 1)
# define use_livox
if(USE_LIVOX)
  add_definitions(-DUSE_LIVOX)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(octomap_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Eigen3)
if(USE_LIVOX)
  find_package(livox_ros_driver2 REQUIRED)
endif()

include_directories(${OpenMP_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(icp_node src/icp_node.cpp)
ament_target_dependencies(icp_node rclcpp std_msgs  geometry_msgs  tf2 tf2_ros tf2_geometry_msgs pcl_ros )
if(USE_LIVOX)
  ament_target_dependencies(icp_node livox_ros_driver2)
endif()

add_executable(transform_publisher src/transform_publisher.cpp)
ament_target_dependencies(transform_publisher rclcpp std_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs pcl_ros)


add_executable(sac_ia_gicp
  src/sac_ia_gicp.cpp
)
ament_target_dependencies(sac_ia_gicp  rclcpp  std_msgs  PCL  pcl_ros  Eigen3)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install nodes
install(
  TARGETS icp_node transform_publisher sac_ia_gicp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
