#pragma once

#include <pix_nina_driver/Byte.hpp>
#include <iostream>

class RawSignalsSteeringAndSpeed {
public:
    static const uint32_t ID = 0x4A;
    RawSignalsSteeringAndSpeed();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    double uint8_vehicle_speed_;
    int uint16_poti_throttle_cmd_;
    int int16_steering_velocity_cmd_;
    double int16_steering_velocity_;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 48, 'is_signed_var': False, 'len': 8, 'name': 'uint8_vehicle_speed', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|25.5]', 'physical_unit': '', 'precision': 0.1, 'type': 'double'}
  double uint8vehiclespeed();

  // config detail: {'bit': 32, 'is_signed_var': False, 'len': 16, 'name': 'uint16_poti_throttle_cmd', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|65535]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int uint16potithrottlecmd();

  // config detail: {'bit': 16, 'is_signed_var': True, 'len': 16, 'name': 'int16_steering_velocity_cmd', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|65535]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int int16steeringvelocitycmd();

  // config detail: {'bit': 0, 'is_signed_var': True, 'len': 16, 'name': 'int16_steering_velocity', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 0.1, 'type': 'double'}
  double int16steeringvelocity();
};



