cmake_minimum_required(VERSION 3.8)
project(rpm_getter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


pkg_check_modules(SERIAL libserial)

include_directories(include)

add_executable(Get_Rpm src/rpm_editor.cpp)
ament_target_dependencies(Get_Rpm rclcpp)
ament_target_dependencies(Get_Rpm rclcpp std_msgs)

install(TARGETS
Get_Rpm
DESTINATION lib/${PROJECT_NAME}
)

ament_package()
