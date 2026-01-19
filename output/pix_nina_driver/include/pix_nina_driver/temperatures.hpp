#pragma once

#include <pix_nina_driver/Byte.hpp>
#include <iostream>

class Temperatures {
public:
    static const uint32_t ID = 0x5;
    Temperatures();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    int steering_motor_inverter_temp_;
    int steering_motor_temp_;
    int curtis_controller_temp_;
    int curtis_motor_temp_;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 24, 'is_signed_var': True, 'len': 8, 'name': 'steering_motor_inverter_temp', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '°C', 'precision': 1.0, 'type': 'int'}
  int steeringmotorinvertertemp();

  // config detail: {'bit': 16, 'is_signed_var': True, 'len': 8, 'name': 'steering_motor_temp', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '°C', 'precision': 1.0, 'type': 'int'}
  int steeringmotortemp();

  // config detail: {'bit': 8, 'is_signed_var': True, 'len': 8, 'name': 'curtis_controller_temp', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '°C', 'precision': 1.0, 'type': 'int'}
  int curtiscontrollertemp();

  // config detail: {'bit': 0, 'is_signed_var': True, 'len': 8, 'name': 'curtis_motor_temp', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '°C', 'precision': 1.0, 'type': 'int'}
  int curtismotortemp();
};



