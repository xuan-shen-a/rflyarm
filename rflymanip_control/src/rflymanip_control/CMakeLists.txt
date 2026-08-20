cmake_minimum_required(VERSION 3.8)
project(rflymanip_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_library(ik_solver_lib
  src/ik_solver.cpp
)
target_include_directories(ik_solver_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(quintic_trajectory_lib
  src/quintic_trajectory.cpp
)
target_include_directories(quintic_trajectory_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(ik_node
  src/ik_node.cpp
)
target_link_libraries(ik_node
  ik_solver_lib
  quintic_trajectory_lib
)
ament_target_dependencies(ik_node
  rclcpp
  sensor_msgs
  std_msgs
)

add_executable(manip_controller_node
  src/manip_controller_node.cpp
)
target_link_libraries(manip_controller_node
  ik_solver_lib
  quintic_trajectory_lib
)
ament_target_dependencies(manip_controller_node
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS
  ik_node
  manip_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ik_solver_lib
  quintic_trajectory_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

