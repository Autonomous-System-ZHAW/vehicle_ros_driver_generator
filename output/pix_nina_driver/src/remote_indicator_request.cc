#include <pix_nina_driver/remote_indicator_request.hpp>

int32_t RemoteIndicatorRequest::ID = 0x10;

// public
RemoteIndicatorRequest::RemoteIndicatorRequest() { Reset(); }

void RemoteIndicatorRequest::UpdateData(int remote_status_light_blink_rate_req, int remote_status_light_color_req, bool remote_horn_req, bool remote_light_brake_req, bool remote_blink_left_req, bool remote_blink_right_req, bool remote_brights_on_req, bool remote_light_reverse_req, bool remote_lights_turn_left_req, bool remote_lights_turn_right_req) {
  set_p_remote_status_light_blink_rate_req(remote_status_light_blink_rate_req);
  set_p_remote_status_light_color_req(remote_status_light_color_req);
  set_p_remote_horn_req(remote_horn_req);
  set_p_remote_light_brake_req(remote_light_brake_req);
  set_p_remote_blink_left_req(remote_blink_left_req);
  set_p_remote_blink_right_req(remote_blink_right_req);
  set_p_remote_brights_on_req(remote_brights_on_req);
  set_p_remote_light_reverse_req(remote_light_reverse_req);
  set_p_remote_lights_turn_left_req(remote_lights_turn_left_req);
  set_p_remote_lights_turn_right_req(remote_lights_turn_right_req);
}

void RemoteIndicatorRequest::Reset() {
  // TODO(All) :  you should check this manually
  for(uint8_t i=0;i<8;i++)
  {
    data[i] = 0;
  }
}

uint8_t * RemoteIndicatorRequest::get_data()
{
  return data;
}



// config detail: {'bit': 10, 'is_signed_var': False, 'len': 2, 'name': 'remote_status_light_blink_rate_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void RemoteIndicatorRequest::set_p_remote_status_light_blink_rate_req(int remote_status_light_blink_rate_req) {
  // remote_status_light_blink_rate_req = ProtocolData::BoundedValue(0, 0, remote_status_light_blink_rate_req);
  int x = remote_status_light_blink_rate_req;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 2, 2);
  data[1] += to_set.return_byte_t();
  
}

// config detail: {'bit': 8, 'is_signed_var': False, 'len': 2, 'name': 'remote_status_light_color_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void RemoteIndicatorRequest::set_p_remote_status_light_color_req(int remote_status_light_color_req) {
  // remote_status_light_color_req = ProtocolData::BoundedValue(0, 0, remote_status_light_color_req);
  int x = remote_status_light_color_req;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 2);
  data[1] += to_set.return_byte_t();
  
}

// config detail: {'bit': 7, 'is_signed_var': False, 'len': 1, 'name': 'remote_horn_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void RemoteIndicatorRequest::set_p_remote_horn_req(bool remote_horn_req) {
  int x = remote_horn_req;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 7, 1);
  data[0] += to_set.return_byte_t();
  
}

// config detail: {'bit': 6, 'is_signed_var': False, 'len': 1, 'name': 'remote_light_brake_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void RemoteIndicatorRequest::set_p_remote_light_brake_req(bool remote_light_brake_req) {
  int x = remote_light_brake_req;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 6, 1);
  data[0] += to_set.return_byte_t();
  
}

// config detail: {'bit': 5, 'is_signed_var': False, 'len': 1, 'name': 'remote_blink_left_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void RemoteIndicatorRequest::set_p_remote_blink_left_req(bool remote_blink_left_req) {
  int x = remote_blink_left_req;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 5, 1);
  data[0] += to_set.return_byte_t();
  
}

// config detail: {'bit': 4, 'is_signed_var': False, 'len': 1, 'name': 'remote_blink_right_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void RemoteIndicatorRequest::set_p_remote_blink_right_req(bool remote_blink_right_req) {
  int x = remote_blink_right_req;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 4, 1);
  data[0] += to_set.return_byte_t();
  
}

// config detail: {'bit': 3, 'is_signed_var': False, 'len': 1, 'name': 'remote_brights_on_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void RemoteIndicatorRequest::set_p_remote_brights_on_req(bool remote_brights_on_req) {
  int x = remote_brights_on_req;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 3, 1);
  data[0] += to_set.return_byte_t();
  
}

// config detail: {'bit': 2, 'is_signed_var': False, 'len': 1, 'name': 'remote_light_reverse_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void RemoteIndicatorRequest::set_p_remote_light_reverse_req(bool remote_light_reverse_req) {
  int x = remote_light_reverse_req;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 2, 1);
  data[0] += to_set.return_byte_t();
  
}

// config detail: {'bit': 1, 'is_signed_var': False, 'len': 1, 'name': 'remote_lights_turn_left_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void RemoteIndicatorRequest::set_p_remote_lights_turn_left_req(bool remote_lights_turn_left_req) {
  int x = remote_lights_turn_left_req;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 1, 1);
  data[0] += to_set.return_byte_t();
  
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'remote_lights_turn_right_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void RemoteIndicatorRequest::set_p_remote_lights_turn_right_req(bool remote_lights_turn_right_req) {
  int x = remote_lights_turn_right_req;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 1);
  data[0] += to_set.return_byte_t();
  
}


