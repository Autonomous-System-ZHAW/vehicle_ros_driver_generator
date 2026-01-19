#pragma once

#include <pix_nina_driver/Byte.hpp>
#include <iostream>

class RawSignalSteeringForce {
public:
    static const uint32_t ID = 0x42;
    RawSignalSteeringForce();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    int uint32_signal_steering_force_b_;
    int uint32_signal_steering_force_a_;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 32, 'is_signed_var': False, 'len': 32, 'name': 'uint32_signal_steering_force_b', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int uint32signalsteeringforceb();

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 32, 'name': 'uint32_signal_steering_force_a', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int uint32signalsteeringforcea();
};



