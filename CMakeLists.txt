cmake_minimum_required(VERSION 3.20)
project(sinsei_umiusi_control)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# default compiler flags
add_compile_options(
  -Wall
  -Wextra
  -Wpedantic

  $<$<CONFIG:Release>:-O3>
  $<$<CONFIG:Debug>:-O0>
)

# C/C++ version required by ROS2 jazzy
# require C99
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
# require C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set sccache
find_program(SCCACHE_EXE sccache)
if(SCCACHE_EXE)
  set(CMAKE_C_COMPILER_LAUNCHER ${SCCACHE_EXE})
  set(CMAKE_CXX_COMPILER_LAUNCHER ${SCCACHE_EXE})
endif()

# for clang-tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# serach for pigpio library
find_library(PIGPIO_LIBRARY pigpiod_if2)
if(NOT PIGPIO_LIBRARY)
  message(FATAL_ERROR "pigpiod_if2 library not found. Please install pigpio.")
endif()

# search for Eigen3
find_package(Eigen3)
if(NOT Eigen3_FOUND)
  message(FATAL_ERROR "Eigen3 not found. Please install Eigen3.")
endif()

# 本来は`msg`や`srv`専用にパッケージを分けるべきだが、リポジトリを跨ぎたくないためここに置いている。
rosidl_generate_interfaces(${PROJECT_NAME}_msgs
  "msg/EscState.msg"  
  "msg/HeadlightsOutput.msg"
  "msg/HighPowerCircuitHealth.msg"
  "msg/ImuState.msg"
  "msg/IndicatorLedOutput.msg"
  "msg/LedTapeOutput.msg"
  "msg/LowPowerCircuitHealth.msg"
  "msg/MainPowerEnabled.msg"
  "msg/MainPowerOutput.msg"
  "msg/Target.msg"
  "msg/ThrusterEnabled.msg"
  "msg/ThrusterEnabledAll.msg"
  "msg/ThrusterOutput.msg"
  "msg/ThrusterOutputAll.msg"
  "msg/ThrusterState.msg"
  "msg/ThrusterStateAll.msg"

  DEPENDENCIES
    std_msgs
    geometry_msgs

  LIBRARY_NAME
    ${PROJECT_NAME}
)
ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_msgs rosidl_typesupport_cpp)

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
  src/${PROJECT_NAME}/hardware_model/thruster_direct/esc_direct_model.cpp
  src/${PROJECT_NAME}/hardware_model/thruster_direct/servo_direct_model.cpp
  src/${PROJECT_NAME}/hardware_model/can/vesc_model.cpp
  src/${PROJECT_NAME}/hardware_model/imu/bno055_model.cpp

  src/${PROJECT_NAME}/hardware_model/impl/pigpio.cpp
  src/${PROJECT_NAME}/hardware_model/impl/linux_can.cpp
)
target_link_libraries(${PROJECT_NAME}_hardware
  ${PIGPIO_LIBRARY}
)

ament_auto_add_library(${PROJECT_NAME}_controller SHARED
  src/${PROJECT_NAME}/controller/gate_controller.cpp
  src/${PROJECT_NAME}/controller/attitude_controller.cpp
  src/${PROJECT_NAME}/controller/thruster_controller.cpp
  src/${PROJECT_NAME}/util/interface_accessor.cpp
)
target_link_libraries(${PROJECT_NAME}_controller
  ${cpp_typesupport_target}
)
target_include_directories(${PROJECT_NAME}_controller
  PRIVATE
    ${Eigen3_INCLUDE_DIRS}
)

pluginlib_export_plugin_description_file(hardware_interface plugin/umiusi_hardware_plugin.xml)
pluginlib_export_plugin_description_file(controller_interface plugin/umiusi_controller_plugin.xml)

ament_python_install_package(${PROJECT_NAME}_python)

install(PROGRAMS
  # src/${PROJECT_NAME}/camera/raspi_camera.py
  src/${PROJECT_NAME}/camera/usb_camera
  src/${PROJECT_NAME}/camera/usb_camera.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(
    pytest test/python
    TIMEOUT 120
  )
  ament_auto_add_gmock(test_hardware_model
    test/cpp/hardware_model/headlights.cpp
    # test/cpp/hardware_model/imu.cpp TODO: 未実装
    test/cpp/hardware_model/imu/bno055.cpp
    test/cpp/hardware_model/indicator_led.cpp
    test/cpp/hardware_model/can.cpp
    test/cpp/hardware_model/can/vesc.cpp
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
  params
  launch
)

# ! 以下、後ろの方に書かないと無限ループに陥るので注意
# ! ref: https://stackoverflow.com/questions/32306648/changing-compiler-with-cmake-creates-infinite-loop

# set clang as the default compiler if available
# TODO: gcovまわりのバグが解消できたら有効化
# find_program(CLANG_EXE clang)
# if(CLANG_EXE)
#   set(CMAKE_C_COMPILER ${CLANG_EXE})
#   set(CMAKE_CXX_COMPILER ${CLANG_EXE}++)
#   set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fuse-ld=lld")
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fuse-ld=lld")
# else()
  set(CMAKE_C_COMPILER gcc)
  set(CMAKE_CXX_COMPILER g++)
# endif()
