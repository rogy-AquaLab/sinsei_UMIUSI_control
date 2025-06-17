cmake_minimum_required(VERSION 3.8)
project(sinsei_umiusi_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_hardware SHARED
  src/${PROJECT_NAME}/hardware/can.cpp
  src/${PROJECT_NAME}/hardware/headlights.cpp
  src/${PROJECT_NAME}/hardware/imu.cpp
  src/${PROJECT_NAME}/hardware/indicator_led.cpp
  src/${PROJECT_NAME}/hardware/raspi_camera.cpp
  src/${PROJECT_NAME}/hardware/esc_direct.cpp
  src/${PROJECT_NAME}/hardware/servo_direct.cpp
  src/${PROJECT_NAME}/hardware/usb_camera.cpp
  include/${PROJECT_NAME}/hardware/can.hpp
  include/${PROJECT_NAME}/hardware/headlights.hpp
  include/${PROJECT_NAME}/hardware/imu.hpp
  include/${PROJECT_NAME}/hardware/indicator_led.hpp
  include/${PROJECT_NAME}/hardware/raspi_camera.hpp
  include/${PROJECT_NAME}/hardware/esc_direct.hpp
  include/${PROJECT_NAME}/hardware/servo_direct.hpp
  include/${PROJECT_NAME}/hardware/usb_camera.hpp
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
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # the following line skips the linter which checks for cmake files
  set(ament_cmake_lint_cmake_FOUND TRUE)
  # the following line skips uncrustify (replaced with clang_format)
  set(ament_cmake_uncrustify_FOUND TRUE)

  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")

  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(pytest test)
endif()

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  urdf
  config
  launch
)