cmake_minimum_required(VERSION 3.8)
project(motion_operator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(humanoid_interfaces REQUIRED)
find_package(dynamixel_hardware_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(motion_operator_node
  src/main.cpp
  src/motion_operator_node.cpp
  src/motion_scheduler.cpp
  src/motion_num_resolver.cpp
  src/utils/motion_operator_utils.cpp
  src/utils/motion_param_loader.cpp
)

target_include_directories(motion_operator_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(motion_operator_node
  yaml-cpp
)

ament_target_dependencies(motion_operator_node
  ament_index_cpp
  rclcpp
  rclcpp_lifecycle
  std_msgs
  humanoid_interfaces
  dynamixel_hardware_msgs
)

install(
  TARGETS motion_operator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config motion
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
