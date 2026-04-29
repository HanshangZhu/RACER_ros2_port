cmake_minimum_required(VERSION 3.14)
project(so3_quadrotor_simulator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(uav_utils REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Armadillo REQUIRED)

include_directories(
  include
  include/ode
  ${EIGEN3_INCLUDE_DIRS}
  ${ARMADILLO_INCLUDE_DIRS}
)

add_library(quadrotor_dynamics src/dynamics/Quadrotor.cpp)
target_include_directories(quadrotor_dynamics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ode>
)
target_link_libraries(quadrotor_dynamics ${ARMADILLO_LIBRARIES})

add_executable(quadrotor_simulator_so3 src/quadrotor_simulator_so3.cpp)
ament_target_dependencies(quadrotor_simulator_so3
  rclcpp nav_msgs sensor_msgs geometry_msgs quadrotor_msgs uav_utils)
target_link_libraries(quadrotor_simulator_so3 quadrotor_dynamics)

install(TARGETS quadrotor_simulator_so3 DESTINATION lib/${PROJECT_NAME})
install(TARGETS quadrotor_dynamics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
