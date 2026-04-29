cmake_minimum_required(VERSION 3.14)
project(multi_map_server)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(eigen3_cmake_module REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pose_utils REQUIRED)
find_package(quadrotor_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Armadillo REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/VerticalOccupancyGridList.msg
  msg/SparseMap3D.msg
  msg/MultiSparseMap3D.msg
  msg/MultiOccupancyGrid.msg
  DEPENDENCIES std_msgs geometry_msgs nav_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

# -------- multi_map_visualization executable --------
add_executable(multi_map_visualization src/multi_map_visualization.cc)

target_include_directories(multi_map_visualization PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${ARMADILLO_INCLUDE_DIRS}
)

ament_target_dependencies(multi_map_visualization
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  tf2
  pcl_conversions
  pcl_ros
  pose_utils
  quadrotor_msgs
)

target_link_libraries(multi_map_visualization
  ${cpp_typesupport_target}
  ${PCL_LIBRARIES}
  ${ARMADILLO_LIBRARIES}
)

install(TARGETS multi_map_visualization
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(
  rosidl_default_runtime
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  tf2
  pcl_conversions
  pcl_ros
  pose_utils
  quadrotor_msgs
  Eigen3
)
ament_package()
