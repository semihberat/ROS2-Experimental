cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(minimal_node src/minimal_node.cpp)
ament_target_dependencies(minimal_node rclcpp)

add_executable(oop_cpp_template src/oop_cpp_template.cpp)
ament_target_dependencies(oop_cpp_template rclcpp)

install(TARGETS
  minimal_node
  oop_cpp_template
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
