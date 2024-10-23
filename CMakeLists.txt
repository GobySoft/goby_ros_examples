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
)

add_library(goby_ros_examples_json SHARED src/example_json.cpp)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS include/goby_ros_examples/nav.proto)
add_library(goby_ros_examples_protobuf SHARED src/example_protobuf.cpp ${PROTO_SRCS} ${PROTO_HDRS})


ament_target_dependencies(goby_ros_examples_json goby_ros_gateway std_msgs rclcpp)
ament_target_dependencies(goby_ros_examples_protobuf goby_ros_gateway std_msgs rclcpp)


rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(goby_ros_examples_protobuf protobuf::libprotobuf ${cpp_typesupport_target})
target_include_directories(goby_ros_examples_protobuf
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/..>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  )

install(TARGETS
  goby_ros_examples_json
  goby_ros_examples_protobuf
  DESTINATION lib/${PROJECT_NAME})

#ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/nav_json_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
