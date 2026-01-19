#include <pix_nina_driver/scaled_signals.hpp>


ScaledSignals::ScaledSignals() {}

void ScaledSignals::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void ScaledSignals::Parse() {
  throttle_signal_ = throttlesignal();
  steering_motor_speed_cmd_ = steeringmotorspeedcmd();
  curtis_speed_cmd_ = curtisspeedcmd();
  steering_torque_signal_ = steeringtorquesignal();
  steering_velocity_signal_ = steeringvelocitysignal();
  brake_signal_ = brakesignal();
}


// config detail: {'bit': 24, 'is_signed_var': False, 'len': 8, 'name': 'throttle_signal', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int ScaledSignals::throttlesignal() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 40, 'is_signed_var': True, 'len': 8, 'name': 'steering_motor_speed_cmd', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '%', 'precision': 1.0, 'type': 'int'}
int ScaledSignals::steeringmotorspeedcmd() {
  Byte t0(*(bytes + 5));
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  int ret = x;
  return ret;
}

// config detail: {'bit': 32, 'is_signed_var': False, 'len': 8, 'name': 'curtis_speed_cmd', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int ScaledSignals::curtisspeedcmd() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 16, 'is_signed_var': True, 'len': 8, 'name': 'steering_torque_signal', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int ScaledSignals::steeringtorquesignal() {
  Byte t0(*(bytes + 2));
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  int ret = x;
  return ret;
}

// config detail: {'bit': 8, 'is_signed_var': True, 'len': 8, 'name': 'steering_velocity_signal', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-128|127]', 'physical_unit': '%', 'precision': 1.0, 'type': 'int'}
int ScaledSignals::steeringvelocitysignal() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  int ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 8, 'name': 'brake_signal', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '%', 'precision': 1.0, 'type': 'int'}
int ScaledSignals::brakesignal() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

