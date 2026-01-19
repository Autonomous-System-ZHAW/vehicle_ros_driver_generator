cmake_minimum_required(VERSION 3.5)
project(pix_nina_driver)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

ament_auto_add_executable(${PROJECT_NAME}_control_command_node
  src/remote_application_toggle_request.cc
src/remote_indicator_request.cc
src/remote_drive_request.cc

  src/Byte.cc
  src/control_command.cpp
  src/control_command_node.cpp
)

ament_auto_add_executable(${PROJECT_NAME}_report_parser_node
  src/general_vehicle_status.cc
src/steering_and_speed.cc
src/u_s_sensor_front.cc
src/u_s_sensor_rear.cc
src/battery_status.cc
src/raw_signal_throttle.cc
src/raw_signal_break.cc
src/raw_signal_steering_force.cc
src/raw_signal_steering_position.cc
src/scaled_signals.cc
src/raw_signals_steering_and_speed.cc
src/temperatures.cc
src/v_e_c_t_o_r___i_n_d_e_p_e_n_d_e_n_t__s_i_g__m_s_g.cc

  src/Byte.cc
  src/report_parser.cpp
  src/report_parser_node.cpp
)

# install
ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
  # config
)