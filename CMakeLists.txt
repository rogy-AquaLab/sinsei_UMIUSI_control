cmake_minimum_required(VERSION 3.8)
project(sinsei_umiusi_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# require C99
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
# require C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# for clang-tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_hardware SHARED
  src/${PROJECT_NAME}/hardware/can.cpp
  src/${PROJECT_NAME}/hardware/headlights.cpp
  src/${PROJECT_NAME}/hardware/imu.cpp
  src/${PROJECT_NAME}/hardware/indicator_led.cpp
  src/${PROJECT_NAME}/hardware/thruster_direct/esc_direct.cpp
  src/${PROJECT_NAME}/hardware/thruster_direct/servo_direct.cpp

  src/${PROJECT_NAME}/hardware_model/can_model.cpp
  src/${PROJECT_NAME}/hardware_model/headlights_model.cpp
  src/${PROJECT_NAME}/hardware_model/imu_model.cpp
  src/${PROJECT_NAME}/hardware_model/indicator_led_model.cpp
  # src/${PROJECT_NAME}/hardware_model/thruster_direct/esc_direct_model.cpp
  # src/${PROJECT_NAME}/hardware_model/thruster_direct/servo_direct_model.cpp
  src/${PROJECT_NAME}/hardware_model/can/vesc_model.cpp

  include/${PROJECT_NAME}/hardware/can.hpp
  include/${PROJECT_NAME}/hardware/headlights.hpp
  include/${PROJECT_NAME}/hardware/imu.hpp
  include/${PROJECT_NAME}/hardware/indicator_led.hpp
  include/${PROJECT_NAME}/hardware/thruster_direct/esc_direct.hpp
  include/${PROJECT_NAME}/hardware/thruster_direct/servo_direct.hpp

  include/${PROJECT_NAME}/hardware_model/can_model.hpp
  include/${PROJECT_NAME}/hardware_model/headlights_model.hpp
  include/${PROJECT_NAME}/hardware_model/imu_model.hpp
  include/${PROJECT_NAME}/hardware_model/indicator_led_model.hpp
  # include/${PROJECT_NAME}/hardware_model/thruster_direct/esc_direct_model.hpp
  # include/${PROJECT_NAME}/hardware_model/thruster_direct/servo_direct_model.hpp
  include/${PROJECT_NAME}/hardware_model/can/vesc_model.hpp

  src/${PROJECT_NAME}/util/pigpio.cpp
  src/${PROJECT_NAME}/util/linux_can.cpp
)

find_library(PIGPIO_LIBRARY pigpiod_if2)
target_link_libraries(${PROJECT_NAME}_hardware
  ${PIGPIO_LIBRARY}
)

ament_auto_add_library(${PROJECT_NAME}_controller SHARED
  src/${PROJECT_NAME}/controller/gate_controller.cpp
  src/${PROJECT_NAME}/controller/app_controller.cpp
  src/${PROJECT_NAME}/controller/thruster_controller.cpp
  include/${PROJECT_NAME}/controller/gate_controller.hpp
  include/${PROJECT_NAME}/controller/app_controller.hpp
  include/${PROJECT_NAME}/controller/thruster_controller.hpp
)

pluginlib_export_plugin_description_file(hardware_interface plugin/umiusi_hardware_plugin.xml)
pluginlib_export_plugin_description_file(controller_interface plugin/umiusi_controller_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(pytest test/python)
  ament_auto_add_gmock(test_hardware_model
    test/cpp/hardware_model/headlights.cpp
    test/cpp/hardware_model/imu.cpp
    test/cpp/hardware_model/indicator_led.cpp
    test/cpp/hardware_model/can.cpp
  )
  target_include_directories(test_hardware_model
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/cpp>
  )
  ament_auto_add_gmock(test_util
    test/cpp/util.cpp
  )
endif()

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  urdf
  config
  launch
)