#pragma once

#include <pix_nina_driver/Byte.hpp>
#include <iostream>

class RawSignalSteeringPosition {
public:
    static const uint32_t ID = 0x43;
    RawSignalSteeringPosition();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    int uint16_encoder_raw_value_b_;
    int uint16_encoder_raw_value_a_;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 16, 'name': 'uint16_encoder_raw_value_b', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int uint16encoderrawvalueb();

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 16, 'name': 'uint16_encoder_raw_value_a', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int uint16encoderrawvaluea();
};



