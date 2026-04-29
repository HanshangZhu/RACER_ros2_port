cmake_minimum_required(VERSION 3.14)
project(traj_utils)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-deprecated-declarations -Wno-unused-variable)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

# Upstream dependencies of planning_visualization. These are Wave 4 (plan_env)
# and Wave 4-late (path_searching, active_perception) packages. If any of them
# is absent from the overlay, the heavy library target is skipped — the
# standalone process_msg executable still builds so that downstream packages
# (plan_manage, exploration_manager) can depend on traj_utils without linking
# against planning_visualization yet.
find_package(bspline QUIET)
find_package(poly_traj QUIET)
find_package(plan_env QUIET)
find_package(path_searching QUIET)
find_package(active_perception QUIET)

set(TRAJ_UTILS_LIB_READY TRUE)
foreach(_dep bspline poly_traj plan_env path_searching active_perception)
  if(NOT ${_dep}_FOUND)
    message(WARNING
      "traj_utils: dependency '${_dep}' not found — planning_visualization "
      "library will be skipped.")
    set(TRAJ_UTILS_LIB_READY FALSE)
  endif()
endforeach()

# -------- planning_visualization library (conditional) --------
if(TRAJ_UTILS_LIB_READY)
  add_library(${PROJECT_NAME}
    src/planning_visualization.cpp
  )
  target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${plan_env_INCLUDE_DIRS}
    ${path_searching_INCLUDE_DIRS}
    ${active_perception_INCLUDE_DIRS}
    ${bspline_INCLUDE_DIRS}
    ${poly_traj_INCLUDE_DIRS}
  )
  ament_target_dependencies(${PROJECT_NAME} PUBLIC
    rclcpp
    std_msgs
    geometry_msgs
    visualization_msgs
    bspline
    poly_traj
    plan_env
    path_searching
    active_perception
  )
  target_link_libraries(${PROJECT_NAME} PUBLIC
    ${PCL_LIBRARIES}
  )

  install(TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  ament_export_libraries(${PROJECT_NAME})
  ament_export_targets(export_${PROJECT_NAME})
endif()

# -------- process_msg executable (always builds) --------
add_executable(process_msg src/process_msg.cpp)
target_include_directories(process_msg PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(process_msg
  rclcpp
  sensor_msgs
  visualization_msgs
  pcl_conversions
)
target_link_libraries(process_msg ${PCL_LIBRARIES})

install(TARGETS process_msg
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  pcl_conversions
  Eigen3
)
ament_package()
