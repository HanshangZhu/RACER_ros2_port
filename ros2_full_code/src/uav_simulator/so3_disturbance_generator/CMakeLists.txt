cmake_minimum_required(VERSION 3.14)
project(so3_disturbance_generator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pose_utils REQUIRED)
find_package(Armadillo REQUIRED)

add_executable(${PROJECT_NAME} src/so3_disturbance_generator.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp geometry_msgs nav_msgs pose_utils)
target_include_directories(${PROJECT_NAME} PRIVATE ${ARMADILLO_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${ARMADILLO_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

ament_package()
