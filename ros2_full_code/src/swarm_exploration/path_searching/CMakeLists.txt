cmake_minimum_required(VERSION 3.14)
project(path_searching)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-deprecated-declarations -Wno-unused-variable
    -Wno-sign-compare -Wno-reorder -Wno-unused-but-set-variable)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(plan_env REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

add_library(path_searching
  src/astar.cpp
  src/astar2.cpp
  src/kinodynamic_astar.cpp
  src/topo_prm.cpp
)

target_include_directories(path_searching PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${plan_env_INCLUDE_DIRS}
)

ament_target_dependencies(path_searching PUBLIC
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  plan_env
)

target_link_libraries(path_searching PUBLIC
  ${PCL_LIBRARIES}
)

install(TARGETS path_searching
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# -------- unicycle primitive unit test (runnable standalone via ros2 run) --
add_executable(test_unicycle_primitive test/test_unicycle_primitive.cpp)
target_include_directories(test_unicycle_primitive PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
)
install(TARGETS test_unicycle_primitive DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(path_searching)
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  plan_env
  Eigen3
)
ament_package()
