cmake_minimum_required(VERSION 3.14)
project(odom_visualization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pose_utils REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Armadillo REQUIRED)

add_executable(${PROJECT_NAME} src/odom_visualization.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp geometry_msgs nav_msgs sensor_msgs visualization_msgs
  tf2_ros pose_utils quadrotor_msgs)
target_include_directories(${PROJECT_NAME} PRIVATE ${EIGEN3_INCLUDE_DIRS} ${ARMADILLO_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${ARMADILLO_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})

ament_package()
