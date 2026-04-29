cmake_minimum_required(VERSION 3.14)
project(so3_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_library(so3_control_lib src/SO3Control.cpp)
target_include_directories(so3_control_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(so3_control_component SHARED src/so3_control_component.cpp)
target_link_libraries(so3_control_component so3_control_lib)
ament_target_dependencies(so3_control_component
  rclcpp rclcpp_components nav_msgs sensor_msgs std_msgs quadrotor_msgs
  tf2 tf2_geometry_msgs)

rclcpp_components_register_nodes(so3_control_component "so3_control::SO3ControlComponent")

install(DIRECTORY include/ DESTINATION include)
install(TARGETS so3_control_lib so3_control_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(so3_control_lib so3_control_component)
ament_export_targets(export_${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
