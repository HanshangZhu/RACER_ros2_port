cmake_minimum_required(VERSION 3.14)
project(plan_manage)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-deprecated-declarations -Wno-unused-variable -Wno-unused-parameter
                      -Wno-sign-compare -Wno-unused-but-set-variable)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(plan_env REQUIRED)
find_package(bspline REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(path_searching REQUIRED)
find_package(active_perception REQUIRED)
find_package(poly_traj REQUIRED)
find_package(traj_utils REQUIRED)
find_package(quadrotor_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

# -------- plan_manage library --------
add_library(${PROJECT_NAME}
  src/planner_manager.cpp
  src/planner_manager_dev.cpp
)

# Transitive include dirs from ament_target_dependencies don't always propagate
# cleanly, so add upstream headers explicitly (mirrors the traj_utils pattern).
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${plan_env_INCLUDE_DIRS}
  ${bspline_INCLUDE_DIRS}
  ${bspline_opt_INCLUDE_DIRS}
  ${path_searching_INCLUDE_DIRS}
  ${active_perception_INCLUDE_DIRS}
  ${poly_traj_INCLUDE_DIRS}
  ${traj_utils_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  pcl_conversions
  plan_env
  bspline
  bspline_opt
  path_searching
  active_perception
  poly_traj
  traj_utils
  quadrotor_msgs
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${PCL_LIBRARIES}
)

# -------- traj_server executable --------
add_executable(traj_server src/traj_server.cpp)

target_include_directories(traj_server PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${bspline_INCLUDE_DIRS}
  ${poly_traj_INCLUDE_DIRS}
  ${active_perception_INCLUDE_DIRS}
  ${plan_env_INCLUDE_DIRS}
  ${traj_utils_INCLUDE_DIRS}
  ${path_searching_INCLUDE_DIRS}
)

ament_target_dependencies(traj_server
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  bspline
  quadrotor_msgs
  poly_traj
  active_perception
  traj_utils
  plan_env
  path_searching
)

# NOTE(ros2-port): explicit plan_env_lib link is required because
# libtraj_utils.a (static) references ObjHistory::global_start_time_ which
# lives in libplan_env_lib.so. find_library resolves the full path from the
# ament overlay — referencing just the bare name fails.
find_library(PLAN_ENV_LIB plan_env_lib HINTS ${plan_env_LIB_DIRS} REQUIRED)
target_link_libraries(traj_server
  ${PCL_LIBRARIES}
  ${PLAN_ENV_LIB}
)

# -------- install --------
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS traj_server
  DESTINATION lib/${PROJECT_NAME}
)

# -------- dev / benchmark test binaries --------
# process_msg, process_msg2, pos_vel_acc, rotation — self-contained (no planner lib).
# compare_topo, local_explore_fsm, opti_node, test_collision_cost — stubbed (see source).
foreach(_test process_msg process_msg2 pos_vel_acc rotation
               compare_topo local_explore_fsm opti_node test_collision_cost)
  add_executable(${_test} test/${_test}.cpp)
  target_include_directories(${_test} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${bspline_INCLUDE_DIRS}
  )
  ament_target_dependencies(${_test}
    rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs
    pcl_conversions quadrotor_msgs bspline)
  target_link_libraries(${_test} ${PCL_LIBRARIES})
  install(TARGETS ${_test} DESTINATION lib/${PROJECT_NAME})
endforeach()

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  pcl_conversions
  plan_env
  bspline
  bspline_opt
  path_searching
  active_perception
  poly_traj
  traj_utils
  quadrotor_msgs
  Eigen3
)
ament_package()
