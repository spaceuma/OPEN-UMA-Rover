cmake_minimum_required(VERSION 3.8)
project(basic_mobile_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch maps models params rviz src worlds
  DESTINATION share/${PROJECT_NAMES}
)

ament_package()
