cmake_minimum_required(VERSION 3.14)
project(bspline)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Bspline.msg
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

# C++ library (math only — does not link against the generated Bspline msg)
add_library(${PROJECT_NAME}_lib src/non_uniform_bspline.cpp)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}_lib rclcpp)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(rosidl_default_runtime builtin_interfaces geometry_msgs std_msgs rclcpp Eigen3)
ament_export_targets(export_${PROJECT_NAME})
ament_package()
