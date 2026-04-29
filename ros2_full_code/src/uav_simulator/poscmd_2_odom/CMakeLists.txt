cmake_minimum_required(VERSION 3.14)
project(poscmd_2_odom)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(${PROJECT_NAME} src/poscmd_2_odom.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp nav_msgs quadrotor_msgs)
target_include_directories(${PROJECT_NAME} PRIVATE ${EIGEN3_INCLUDE_DIRS})

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

ament_package()
