cmake_minimum_required(VERSION 3.8)
project(omx_hardware_interface)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()


## YAML-CPP
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
  NAMES yaml_cpp.h
  PATHS ${YAML_CPP_INCLUDE_DIRS}
)
find_library(YAML_CPP_LIBRARY
  NAMES YAML_CPP
  PATHS ${YAML_CPP_LIBRARY_DIRS}
)
link_directories(${YAML_CPP_LIBRARY_DIRS})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(dynamixel_workbench_toolbox REQUIRED)
find_package(controller_manager REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${YAML_CPP_INCLUDE_DIRS}
)
# add_compile_options(-Wno-shadow)
add_executable(temp src/temp.cpp src/omx_hardware_interface.cpp)
ament_target_dependencies(
  temp
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  dynamixel_workbench_toolbox
  controller_manager
  yaml-cpp
)
target_link_libraries(temp ${YAML_CPP_LIBRARIES})


install(
  TARGETS
  temp
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
