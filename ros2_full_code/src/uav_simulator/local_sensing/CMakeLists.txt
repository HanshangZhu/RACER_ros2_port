cmake_minimum_required(VERSION 3.14)
project(local_sensing)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Keep GPU path off for the ROS 2 port. Flip to TRUE and add the CUDA + svo
# plumbing back once there is a confirmed need.
set(ENABLE_CUDA FALSE)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

if(ENABLE_CUDA)
  # TODO(ros2-port): re-enable the CUDA depth-render path. Requires:
  #   - find_package(CUDA REQUIRED)
  #   - CUDA_ADD_LIBRARY(depth_render_cuda src/depth_render.cu)
  #   - build src/pcl_render_node.cpp (the GPU variant) instead of
  #     src/pointcloud_render_node.cpp
  message(FATAL_ERROR "CUDA path not yet ported to ROS 2 for local_sensing.")
endif()

# Non-CUDA path: software point-cloud render.
add_executable(pcl_render_node src/pointcloud_render_node.cpp)

ament_target_dependencies(pcl_render_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  quadrotor_msgs
  tf2_ros
  pcl_conversions
  pcl_ros
  cv_bridge
  image_transport
)

target_include_directories(pcl_render_node PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(pcl_render_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

install(TARGETS pcl_render_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
