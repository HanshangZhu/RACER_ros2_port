cmake_minimum_required(VERSION 3.14)
project(lkh_tsp_solver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  srv/SolveTSP.srv
)

# The C++ node shells out to the external `/usr/local/bin/LKH` binary rather
# than bundling the LKH algorithm C sources. See src/tsp_node.cpp for details.
add_executable(tsp_node src/tsp_node.cpp)
ament_target_dependencies(tsp_node rclcpp)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(tsp_node ${cpp_typesupport_target})

install(TARGETS tsp_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime rclcpp)
ament_package()
