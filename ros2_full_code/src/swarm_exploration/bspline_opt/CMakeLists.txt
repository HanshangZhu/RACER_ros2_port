cmake_minimum_required(VERSION 3.14)
project(bspline_opt)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-deprecated-declarations -Wno-sign-compare -Wno-unused-variable)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(plan_env REQUIRED)
find_package(bspline REQUIRED)

find_package(Eigen3 REQUIRED)

# nlopt is vendored outside ament. Canonical location (baked into the Docker
# image) is /usr/local/lib/libnlopt.so + /usr/local/include/nlopt.hpp. When
# building on a host without it, fall back to a workspace-local copy at
# <workspace>/third_party/nlopt (populated once via the apt debs).
set(NLOPT_ROOT "/usr/local")
set(NLOPT_INCLUDE_DIR "${NLOPT_ROOT}/include")
set(NLOPT_LIB "${NLOPT_ROOT}/lib/libnlopt.so")
if(NOT EXISTS "${NLOPT_LIB}")
  set(_ws_root "${CMAKE_CURRENT_SOURCE_DIR}/../../..")
  get_filename_component(_ws_root "${_ws_root}" ABSOLUTE)
  if(EXISTS "${_ws_root}/third_party/nlopt/lib/libnlopt.so")
    set(NLOPT_ROOT "${_ws_root}/third_party/nlopt")
    set(NLOPT_INCLUDE_DIR "${NLOPT_ROOT}/include")
    set(NLOPT_LIB "${NLOPT_ROOT}/lib/libnlopt.so")
    message(STATUS "bspline_opt: using workspace-local nlopt at ${NLOPT_ROOT}")
  endif()
endif()

# -------- bspline_opt library --------
add_library(${PROJECT_NAME} src/bspline_optimizer.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME} PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
  ${plan_env_INCLUDE_DIRS}
  ${bspline_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIR}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  plan_env
  bspline
)

target_link_libraries(${PROJECT_NAME}
  ${NLOPT_LIB}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  plan_env
  bspline
  Eigen3
)
ament_export_targets(export_${PROJECT_NAME})
ament_package()
