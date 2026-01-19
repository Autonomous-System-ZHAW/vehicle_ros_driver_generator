#pragma once

#include <pix_nina_driver/Byte.hpp>
#include <iostream>

class ScaledSignals {
public:
    static const uint32_t ID = 0x48;
    ScaledSignals();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    int throttle_signal_;
    int steering_motor_speed_cmd_;
    int curtis_speed_cmd_;
    int steering_torque_signal_;
    int steering_velocity_signal_;
    int brake_signal_;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 24, 'is_signed_var': False, 'len': 8, 'name': 'throttle_signal', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int throttlesignal();

  // config detail: {'bit': 40, 'is_signed_var': True, 'len': 8, 'name': 'steering_motor_speed_cmd', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '%', 'precision': 1.0, 'type': 'int'}
  int steeringmotorspeedcmd();

  // config detail: {'bit': 32, 'is_signed_var': False, 'len': 8, 'name': 'curtis_speed_cmd', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int curtisspeedcmd();

  // config detail: {'bit': 16, 'is_signed_var': True, 'len': 8, 'name': 'steering_torque_signal', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int steeringtorquesignal();

  // config detail: {'bit': 8, 'is_signed_var': True, 'len': 8, 'name': 'steering_velocity_signal', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-128|127]', 'physical_unit': '%', 'precision': 1.0, 'type': 'int'}
  int steeringvelocitysignal();

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 8, 'name': 'brake_signal', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '%', 'precision': 1.0, 'type': 'int'}
  int brakesignal();
};



