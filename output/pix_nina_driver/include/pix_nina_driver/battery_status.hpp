#pragma once

#include <pix_nina_driver/Byte.hpp>
#include <iostream>

class BatteryStatus {
public:
    static const uint32_t ID = 0x4;
    BatteryStatus();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    int battery_discharge_percent_;
    double battery_current_;
    double battery_voltage_;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 32, 'is_signed_var': False, 'len': 8, 'name': 'battery_discharge_percent', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '%', 'precision': 1.0, 'type': 'int'}
  int batterydischargepercent();

  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 16, 'name': 'battery_current', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': 'A', 'precision': 0.1, 'type': 'double'}
  double batterycurrent();

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 16, 'name': 'battery_voltage', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': 'V', 'precision': 0.01, 'type': 'double'}
  double batteryvoltage();
};



