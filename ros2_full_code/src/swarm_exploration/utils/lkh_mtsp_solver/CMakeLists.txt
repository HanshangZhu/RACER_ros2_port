cmake_minimum_required(VERSION 3.14)
project(lkh_mtsp_solver)

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
  srv/SolveMTSP.srv
)

# Shells out to the external `/usr/local/bin/LKH` binary.
add_executable(mtsp_node src/mtsp_node.cpp)
ament_target_dependencies(mtsp_node rclcpp)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(mtsp_node ${cpp_typesupport_target})

install(TARGETS mtsp_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the resource/ templates (*.par, *.atsp) so launch code can locate
# them via ament_index at runtime if needed.
install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime rclcpp)
ament_package()
