cmake_minimum_required(VERSION 3.14)
project(cmake_utils)

find_package(ament_cmake REQUIRED)

# Install CMake helper files so they can be consumed by downstream packages
# via ament_cmake_package(CONFIG_EXTRAS ...) or manual include(...).
install(DIRECTORY cmake cmake_modules
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
