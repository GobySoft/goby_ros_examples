cmake_minimum_required(VERSION 3.8)
project(goby_ros_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(goby_ros_gateway REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Protobuf REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Nav.msg"
  "msg/ProtobufEncoded.msg"
  "msg/ProtobufJSON.msg"
)

add_library(goby_ros_examples_json SHARED src/plugins/example_json.cpp)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS include/goby_ros_examples/nav.proto)
add_library(goby_ros_examples_protobuf SHARED src/plugins/example_protobuf.cpp ${PROTO_SRCS} ${PROTO_HDRS})

ament_target_dependencies(goby_ros_examples_json goby_ros_gateway std_msgs rclcpp)
ament_target_dependencies(goby_ros_examples_protobuf goby_ros_gateway std_msgs rclcpp)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(goby_ros_examples_protobuf protobuf::libprotobuf ${cpp_typesupport_target})
target_include_directories(goby_ros_examples_protobuf
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/..>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  )

add_executable(ros_protobuf_encoded_publisher src/nodes/ros_protobuf_encoded_publisher.cpp)
ament_target_dependencies(ros_protobuf_encoded_publisher rclcpp)
target_link_libraries(ros_protobuf_encoded_publisher protobuf::libprotobuf ${cpp_typesupport_target} goby_ros_examples_protobuf)
target_include_directories(ros_protobuf_encoded_publisher
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/..>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  )

add_executable(ros_protobuf_encoded_subscriber src/nodes/ros_protobuf_encoded_subscriber.cpp)
ament_target_dependencies(ros_protobuf_encoded_subscriber rclcpp)
target_link_libraries(ros_protobuf_encoded_subscriber protobuf::libprotobuf ${cpp_typesupport_target} goby_ros_examples_protobuf)
target_include_directories(ros_protobuf_encoded_subscriber
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/..>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  )


install(TARGETS
  goby_ros_examples_json
  goby_ros_examples_protobuf
  DESTINATION lib/)

install(TARGETS
  ros_protobuf_encoded_publisher
  ros_protobuf_encoded_subscriber
  DESTINATION lib/${PROJECT_NAME})

#ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/nav_json_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
