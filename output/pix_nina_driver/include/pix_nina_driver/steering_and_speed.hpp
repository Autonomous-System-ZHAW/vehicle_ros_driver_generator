#pragma once

#include <pix_nina_driver/Byte.hpp>
#include <iostream>

class SteeringAndSpeed {
public:
    static const uint32_t ID = 0x3;
    SteeringAndSpeed();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    double vehicle_velocity_requested_;
    double steering_position_requested_;
    double vehicle_velocity_measured_;
    double steering_position_measured_;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 40, 'is_signed_var': True, 'len': 8, 'name': 'vehicle_velocity_requested', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-12.8|12.7]', 'physical_unit': 'm/s', 'precision': 0.1, 'type': 'double'}
  double vehiclevelocityrequested();

  // config detail: {'bit': 16, 'is_signed_var': True, 'len': 16, 'name': 'steering_position_requested', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-163.84|163.835]', 'physical_unit': '%', 'precision': 0.005, 'type': 'double'}
  double steeringpositionrequested();

  // config detail: {'bit': 32, 'is_signed_var': True, 'len': 8, 'name': 'vehicle_velocity_measured', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-12.8|12.7]', 'physical_unit': 'm/s', 'precision': 0.1, 'type': 'double'}
  double vehiclevelocitymeasured();

  // config detail: {'bit': 0, 'is_signed_var': True, 'len': 16, 'name': 'steering_position_measured', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-163.84|163.835]', 'physical_unit': '%', 'precision': 0.005, 'type': 'double'}
  double steeringpositionmeasured();
};



