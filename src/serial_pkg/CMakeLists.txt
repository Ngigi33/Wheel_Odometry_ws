cmake_minimum_required(VERSION 3.8)
project(serial_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
pkg_check_modules(SERIAL libserial)


include_directories(include)

add_executable(Serial_node src/serial_node.cpp)
ament_target_dependencies(Serial_node rclcpp)
ament_target_dependencies(Serial_node rclcpp std_msgs)
ament_target_dependencies(Serial_node rclcpp geometry_msgs)
target_include_directories(Serial_node PUBLIC "${SERIAL_INCLUDE_DIRS}")
target_link_libraries(Serial_node "${SERIAL_LDFLAGS}")

install(TARGETS
Serial_node
DESTINATION lib/${PROJECT_NAME}
)

ament_package()
