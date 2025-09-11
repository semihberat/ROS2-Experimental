cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(std_msgs REQUIRED)


add_executable(minimal_node src/minimal_node.cpp)
ament_target_dependencies(minimal_node rclcpp)

add_executable(oop_cpp_template src/oop_cpp_template.cpp)
ament_target_dependencies(oop_cpp_template rclcpp)

add_executable(publisher_node src/publisher_node.cpp)
ament_target_dependencies(publisher_node rclcpp std_msgs)

add_executable(subscriber_node src/subscriber_node.cpp)
ament_target_dependencies(subscriber_node rclcpp std_msgs)

add_executable(hybrid_node src/hybrid_node.cpp)
ament_target_dependencies(hybrid_node rclcpp std_msgs)

install(TARGETS
  minimal_node
  oop_cpp_template
  publisher_node
  subscriber_node 
  hybrid_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
