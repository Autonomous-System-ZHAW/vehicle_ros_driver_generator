#include <pix_nina_driver/raw_signals_steering_and_speed.hpp>


RawSignalsSteeringAndSpeed::RawSignalsSteeringAndSpeed() {}

void RawSignalsSteeringAndSpeed::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void RawSignalsSteeringAndSpeed::Parse() {
  uint8_vehicle_speed_ = uint8vehiclespeed();
  uint16_poti_throttle_cmd_ = uint16potithrottlecmd();
  int16_steering_velocity_cmd_ = int16steeringvelocitycmd();
  int16_steering_velocity_ = int16steeringvelocity();
}


// config detail: {'bit': 48, 'is_signed_var': False, 'len': 8, 'name': 'uint8_vehicle_speed', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|25.5]', 'physical_unit': '', 'precision': 0.1, 'type': 'double'}
double RawSignalsSteeringAndSpeed::uint8vehiclespeed() {
  Byte t0(*(bytes + 6));
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 32, 'is_signed_var': False, 'len': 16, 'name': 'uint16_poti_throttle_cmd', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|65535]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int RawSignalsSteeringAndSpeed::uint16potithrottlecmd() {
  Byte t0(*(bytes + 5));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 4));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'bit': 16, 'is_signed_var': True, 'len': 16, 'name': 'int16_steering_velocity_cmd', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|65535]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int RawSignalsSteeringAndSpeed::int16steeringvelocitycmd() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 2));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  int ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': True, 'len': 16, 'name': 'int16_steering_velocity', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 0.1, 'type': 'double'}
double RawSignalsSteeringAndSpeed::int16steeringvelocity() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 0));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.100000;
  return ret;
}

