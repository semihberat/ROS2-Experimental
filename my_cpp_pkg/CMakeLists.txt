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

add_executable(number_publisher src/number_counter_assignment/number_publisher.cpp)
ament_target_dependencies(number_publisher rclcpp example_interfaces)

add_executable(number_counter_hybrid src/number_counter_assignment/number_counter_hybrid.cpp)
ament_target_dependencies(number_counter_hybrid rclcpp example_interfaces)

add_executable(server_service src/server_service.cpp)
ament_target_dependencies(server_service rclcpp example_interfaces)

add_executable(client_service src/client_service.cpp)
ament_target_dependencies(client_service rclcpp example_interfaces)

add_executable(reset_counter_client src/number_counter_assignment/reset_counter_client.cpp)
ament_target_dependencies(reset_counter_client rclcpp example_interfaces)


install(TARGETS
  minimal_node
  oop_cpp_template
  publisher_node
  subscriber_node 
  hybrid_node
  number_publisher
  number_counter_hybrid
  server_service
  client_service
  reset_counter_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
