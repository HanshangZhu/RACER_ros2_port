cmake_minimum_required(VERSION 3.14)
project(plan_env)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-deprecated-declarations)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(eigen3_cmake_module REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/IdxList.msg
  msg/ChunkStamps.msg
  msg/ChunkData.msg
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

# -------- plan_env library --------
add_library(plan_env_lib
  src/sdf_map.cpp
  src/map_ros.cpp
  src/multi_map_manager.cpp
  src/edt_environment.cpp
  src/obj_predictor.cpp
  src/raycast.cpp
)

target_include_directories(plan_env_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(plan_env_lib PUBLIC
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  cv_bridge
  image_transport
  tf2
  tf2_ros
  message_filters
  pcl_conversions
  pcl_ros
)

target_link_libraries(plan_env_lib PUBLIC
  ${cpp_typesupport_target}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# -------- obj_generator executable --------
add_executable(obj_generator src/obj_generator.cpp)
target_include_directories(obj_generator PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(obj_generator
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  pcl_conversions
)
target_link_libraries(obj_generator ${PCL_LIBRARIES})

install(TARGETS plan_env_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS obj_generator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(plan_env_lib)
ament_export_dependencies(
  rosidl_default_runtime
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  cv_bridge
  image_transport
  tf2
  tf2_ros
  message_filters
  pcl_conversions
  pcl_ros
  Eigen3
)
ament_package()
