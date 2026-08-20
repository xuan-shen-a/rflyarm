# rflymanip_bridge 2026/1/29版，添加lkmotor相关

cmake_minimum_required(VERSION 3.8)
project(rflymanip_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_library(lkmotor_driver src/lkmotor_driver.cpp)
ament_target_dependencies(lkmotor_driver rclcpp can_msgs)

add_executable(motor_controller_node src/motor_controller_node.cpp)
target_link_libraries(motor_controller_node lkmotor_driver)
ament_target_dependencies(motor_controller_node rclcpp can_msgs std_msgs sensor_msgs)


add_executable(manip_logic_node src/manip_logic_node.cpp)
ament_target_dependencies(manip_logic_node rclcpp sensor_msgs std_msgs)

install(TARGETS 
  lkmotor_driver
  motor_controller_node
  manip_logic_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ── Python 支持 ────────────────────────────────────────────
# 将 servo_sdk/ 安装为可 import 的 Python 包
ament_python_install_package(servo_sdk)

# 将 servo_controller_node.py 注册为可执行脚本
# 安装后可用: ros2 run rflymanip_bridge servo_controller_node.py
install(PROGRAMS
  servo_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()