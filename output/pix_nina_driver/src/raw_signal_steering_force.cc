#include <pix_nina_driver/raw_signal_steering_force.hpp>


RawSignalSteeringForce::RawSignalSteeringForce() {}

void RawSignalSteeringForce::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void RawSignalSteeringForce::Parse() {
  uint32_signal_steering_force_b_ = uint32signalsteeringforceb();
  uint32_signal_steering_force_a_ = uint32signalsteeringforcea();
}


// config detail: {'bit': 32, 'is_signed_var': False, 'len': 32, 'name': 'uint32_signal_steering_force_b', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int RawSignalSteeringForce::uint32signalsteeringforceb() {
  Byte t0(*(bytes + 7));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 6));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(*(bytes + 5));
  t = t2.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t3(*(bytes + 4));
  t = t3.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 32, 'name': 'uint32_signal_steering_force_a', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int RawSignalSteeringForce::uint32signalsteeringforcea() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 2));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(*(bytes + 1));
  t = t2.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t3(*(bytes + 0));
  t = t3.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

