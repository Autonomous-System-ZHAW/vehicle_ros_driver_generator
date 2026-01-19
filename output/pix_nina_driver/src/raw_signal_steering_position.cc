#include <pix_nina_driver/raw_signal_steering_position.hpp>


RawSignalSteeringPosition::RawSignalSteeringPosition() {}

void RawSignalSteeringPosition::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void RawSignalSteeringPosition::Parse() {
  uint16_encoder_raw_value_b_ = uint16encoderrawvalueb();
  uint16_encoder_raw_value_a_ = uint16encoderrawvaluea();
}


// config detail: {'bit': 16, 'is_signed_var': False, 'len': 16, 'name': 'uint16_encoder_raw_value_b', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int RawSignalSteeringPosition::uint16encoderrawvalueb() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 2));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 16, 'name': 'uint16_encoder_raw_value_a', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int RawSignalSteeringPosition::uint16encoderrawvaluea() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 0));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

