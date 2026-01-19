#include <pix_nina_driver/steering_and_speed.hpp>


SteeringAndSpeed::SteeringAndSpeed() {}

void SteeringAndSpeed::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void SteeringAndSpeed::Parse() {
  vehicle_velocity_requested_ = vehiclevelocityrequested();
  steering_position_requested_ = steeringpositionrequested();
  vehicle_velocity_measured_ = vehiclevelocitymeasured();
  steering_position_measured_ = steeringpositionmeasured();
}


// config detail: {'bit': 40, 'is_signed_var': True, 'len': 8, 'name': 'vehicle_velocity_requested', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-12.8|12.7]', 'physical_unit': 'm/s', 'precision': 0.1, 'type': 'double'}
double SteeringAndSpeed::vehiclevelocityrequested() {
  Byte t0(*(bytes + 5));
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 16, 'is_signed_var': True, 'len': 16, 'name': 'steering_position_requested', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-163.84|163.835]', 'physical_unit': '%', 'precision': 0.005, 'type': 'double'}
double SteeringAndSpeed::steeringpositionrequested() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 2));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.005000;
  return ret;
}

// config detail: {'bit': 32, 'is_signed_var': True, 'len': 8, 'name': 'vehicle_velocity_measured', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-12.8|12.7]', 'physical_unit': 'm/s', 'precision': 0.1, 'type': 'double'}
double SteeringAndSpeed::vehiclevelocitymeasured() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': True, 'len': 16, 'name': 'steering_position_measured', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-163.84|163.835]', 'physical_unit': '%', 'precision': 0.005, 'type': 'double'}
double SteeringAndSpeed::steeringpositionmeasured() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 0));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.005000;
  return ret;
}

