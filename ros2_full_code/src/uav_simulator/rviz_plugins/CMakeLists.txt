cmake_minimum_required(VERSION 3.14)
project(rviz_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-deprecated-declarations -Wno-unused-variable)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(tf2 REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

# Currently ported: pose_tool (base) + goal_tool (Goal3DTool).
# Sidelined with .ros1 extensions: aerialmap_display, probmap_display,
# multi_probmap_display, gamelikeinput — see README.md.
set(SRC
  src/pose_tool.cpp
  src/goal_tool.cpp
)

add_library(${PROJECT_NAME} SHARED ${SRC})
target_include_directories(${PROJECT_NAME} PRIVATE src)
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  pluginlib
  rviz_common
  rviz_default_plugins
  rviz_rendering
  tf2)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(FILES plugin_description.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY icons DESTINATION share/${PROJECT_NAME} OPTIONAL)

ament_export_dependencies(rclcpp geometry_msgs pluginlib rviz_common
  rviz_default_plugins rviz_rendering tf2)
ament_package()
