#include <pix_nina_driver/temperatures.hpp>


Temperatures::Temperatures() {}

void Temperatures::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void Temperatures::Parse() {
  steering_motor_inverter_temp_ = steeringmotorinvertertemp();
  steering_motor_temp_ = steeringmotortemp();
  curtis_controller_temp_ = curtiscontrollertemp();
  curtis_motor_temp_ = curtismotortemp();
}


// config detail: {'bit': 24, 'is_signed_var': True, 'len': 8, 'name': 'steering_motor_inverter_temp', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '°C', 'precision': 1.0, 'type': 'int'}
int Temperatures::steeringmotorinvertertemp() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  int ret = x;
  return ret;
}

// config detail: {'bit': 16, 'is_signed_var': True, 'len': 8, 'name': 'steering_motor_temp', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '°C', 'precision': 1.0, 'type': 'int'}
int Temperatures::steeringmotortemp() {
  Byte t0(*(bytes + 2));
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  int ret = x;
  return ret;
}

// config detail: {'bit': 8, 'is_signed_var': True, 'len': 8, 'name': 'curtis_controller_temp', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '°C', 'precision': 1.0, 'type': 'int'}
int Temperatures::curtiscontrollertemp() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  int ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': True, 'len': 8, 'name': 'curtis_motor_temp', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '°C', 'precision': 1.0, 'type': 'int'}
int Temperatures::curtismotortemp() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  int ret = x;
  return ret;
}

