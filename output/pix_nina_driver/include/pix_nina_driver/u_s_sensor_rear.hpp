#pragma once

#include <pix_nina_driver/Byte.hpp>
#include <iostream>

class USSensorRear {
public:
    static const uint32_t ID = 0x31;
    USSensorRear();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    int u_s_sensor8_;
    int u_s_sensor7_;
    int u_s_sensor6_;
    int u_s_sensor5_;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 48, 'is_signed_var': False, 'len': 16, 'name': 'u_s_sensor8', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int ussensor8();

  // config detail: {'bit': 32, 'is_signed_var': False, 'len': 16, 'name': 'u_s_sensor7', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int ussensor7();

  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 16, 'name': 'u_s_sensor6', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int ussensor6();

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 16, 'name': 'u_s_sensor5', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int ussensor5();
};



