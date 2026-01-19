#include <pix_nina_driver/battery_status.hpp>


BatteryStatus::BatteryStatus() {}

void BatteryStatus::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void BatteryStatus::Parse() {
  battery_discharge_percent_ = batterydischargepercent();
  battery_current_ = batterycurrent();
  battery_voltage_ = batteryvoltage();
}


// config detail: {'bit': 32, 'is_signed_var': False, 'len': 8, 'name': 'battery_discharge_percent', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '%', 'precision': 1.0, 'type': 'int'}
int BatteryStatus::batterydischargepercent() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 16, 'is_signed_var': False, 'len': 16, 'name': 'battery_current', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': 'A', 'precision': 0.1, 'type': 'double'}
double BatteryStatus::batterycurrent() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 2));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 16, 'name': 'battery_voltage', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': 'V', 'precision': 0.01, 'type': 'double'}
double BatteryStatus::batteryvoltage() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 0));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.010000;
  return ret;
}

