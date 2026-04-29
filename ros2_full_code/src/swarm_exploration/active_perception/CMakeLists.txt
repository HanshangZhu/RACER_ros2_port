cmake_minimum_required(VERSION 3.14)
project(active_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-deprecated-declarations -Wno-unused-variable
    -Wno-sign-compare -Wno-reorder -Wno-unused-but-set-variable
    -Wno-maybe-uninitialized -Wno-return-type)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(plan_env REQUIRED)
find_package(bspline REQUIRED)
find_package(path_searching REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

# -------- active_perception library --------
add_library(active_perception
  src/frontier_finder.cpp
  src/heading_planner.cpp
  src/perception_utils.cpp
  src/graph_node.cpp
  src/traj_visibility.cpp
  src/uniform_grid.cpp
  src/hgrid.cpp
)

target_include_directories(active_perception PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${plan_env_INCLUDE_DIRS}
  ${bspline_INCLUDE_DIRS}
  ${path_searching_INCLUDE_DIRS}
)

ament_target_dependencies(active_perception PUBLIC
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  sensor_msgs
  pcl_conversions
  plan_env
  bspline
  path_searching
)

target_link_libraries(active_perception PUBLIC
  ${PCL_LIBRARIES}
)

install(TARGETS active_perception
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(active_perception)
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  sensor_msgs
  pcl_conversions
  plan_env
  bspline
  path_searching
  Eigen3
)
ament_package()
