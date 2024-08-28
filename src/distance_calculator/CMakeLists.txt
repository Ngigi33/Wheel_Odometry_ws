cmake_minimum_required(VERSION 3.8)
project(distance_calculator)

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


add_executable(Distance_Calculator src/distance_calculator.cpp)
ament_target_dependencies(Distance_Calculator rclcpp)
ament_target_dependencies(Distance_Calculator rclcpp std_msgs)
ament_target_dependencies(Distance_Calculator rclcpp example_interfaces)
ament_target_dependencies(Distance_Calculator rclcpp nav_msgs)
ament_target_dependencies(Distance_Calculator rclcpp geometry_msgs)
ament_target_dependencies(Distance_Calculator rclcpp tf2)
ament_target_dependencies(Distance_Calculator rclcpp tf2_ros)


install(TARGETS
Distance_Calculator
DESTINATION lib/${PROJECT_NAME}
)

ament_package()
