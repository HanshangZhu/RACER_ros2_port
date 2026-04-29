cmake_minimum_required(VERSION 3.14)
project(exploration_manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-deprecated-declarations -Wno-unused-variable -Wno-unused-parameter
                      -Wno-sign-compare -Wno-unused-but-set-variable -Wno-reorder)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
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
find_package(plan_manage REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(multi_map_server REQUIRED)
find_package(lkh_tsp_solver REQUIRED)
find_package(lkh_mtsp_solver REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

# -------- IDL (messages) --------
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/DeletedGoals.msg
  msg/DroneState.msg
  msg/GridIds.msg
  msg/GridTour.msg
  msg/HGrid.msg
  msg/PairOpt.msg
  msg/PairOptResponse.msg
  msg/SentGoals.msg
  DEPENDENCIES geometry_msgs
)

# -------- exploration_manager library --------
# STATIC: mirrors plan_manage. Several upstream static deps (path_searching,
# bspline_opt) contain multiple definitions across translation units and are
# not built -fPIC, so linking them into a shared library fails. Keeping this
# archive static pushes resolution of those symbols to the executable link
# step, which matches the pattern used by plan_manage / traj_server.
add_library(exploration_manager_lib STATIC
  src/fast_exploration_manager.cpp
  src/fast_exploration_fsm.cpp
)

target_include_directories(exploration_manager_lib PUBLIC
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
  ${plan_manage_INCLUDE_DIRS}
  ${quadrotor_msgs_INCLUDE_DIRS}
  ${multi_map_server_INCLUDE_DIRS}
  ${lkh_tsp_solver_INCLUDE_DIRS}
  ${lkh_mtsp_solver_INCLUDE_DIRS}
)

ament_target_dependencies(exploration_manager_lib PUBLIC
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
  plan_manage
  quadrotor_msgs
  multi_map_server
  lkh_tsp_solver
  lkh_mtsp_solver
)

target_link_libraries(exploration_manager_lib PUBLIC
  ${PCL_LIBRARIES}
)

# Link our own IDL cpp typesupport so we can use exploration_manager/msg/*.hpp
# inside the library.
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(exploration_manager_lib PUBLIC ${cpp_typesupport_target})

# traj_utils is a static archive that references symbols from plan_env_lib
# (ObjHistory::global_start_time_); the same workaround as in plan_manage
# applies here so the final executables link cleanly.
find_library(PLAN_ENV_LIB plan_env_lib HINTS ${plan_env_LIB_DIRS} REQUIRED)
target_link_libraries(exploration_manager_lib PUBLIC ${PLAN_ENV_LIB})

# -------- executables --------
# NOTE(ros2-port): libpath_searching.a ships both astar.cpp and astar2.cpp
# which each define a distinct `class Astar` with the same symbol names in
# the fast_planner namespace. Upstream never links both into the same
# binary, but the ROS 2 port compiles them together. traj_server avoids the
# collision by not referencing Astar; exploration_node uses the Astar API
# directly, so we enable --allow-multiple-definition to let the linker keep
# the first definition (from astar2.cpp, which is the one the ROS 2 headers
# declare via path_searching/astar2.h). Same workaround applied to
# ground_node.
add_executable(exploration_node src/exploration_node.cpp)
target_link_libraries(exploration_node exploration_manager_lib)
target_link_options(exploration_node PRIVATE "-Wl,--allow-multiple-definition")

add_executable(ground_node src/ground_node.cpp)
target_link_libraries(ground_node exploration_manager_lib)
target_link_options(ground_node PRIVATE "-Wl,--allow-multiple-definition")

# -------- install --------
install(TARGETS exploration_manager_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS exploration_node ground_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config resource
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_export_include_directories(include)
ament_export_libraries(exploration_manager_lib)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rosidl_default_runtime
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
  plan_manage
  quadrotor_msgs
  multi_map_server
  lkh_tsp_solver
  lkh_mtsp_solver
  Eigen3
)
ament_package()
