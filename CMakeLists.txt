cmake_minimum_required(VERSION 3.8)
project(omx_hardware_interface)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(dynamixel_workbench_toolbox REQUIRED)

include_directories(include)

add_executable(temp src/temp.cpp src/omx_hardware_interface.cpp)
ament_target_dependencies(
  temp
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  dynamixel_workbench_toolbox
)

install(
  TARGETS
  temp
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
