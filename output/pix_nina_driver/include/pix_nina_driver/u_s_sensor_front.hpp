#pragma once

#include <pix_nina_driver/Byte.hpp>
#include <iostream>

class USSensorFront {
public:
    static const uint32_t ID = 0x30;
    USSensorFront();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    int u_s_sensor4_;
    int u_s_sensor3_;
    int u_s_sensor2_;
    int u_s_sensor1_;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 48, 'is_signed_var': False, 'len': 16, 'name': 'u_s_sensor4', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int ussensor4();

  // config detail: {'bit': 32, 'is_signed_var': False, 'len': 16, 'name': 'u_s_sensor3', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int ussensor3();

  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 16, 'name': 'u_s_sensor2', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int ussensor2();

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 16, 'name': 'u_s_sensor1', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int ussensor1();
};



