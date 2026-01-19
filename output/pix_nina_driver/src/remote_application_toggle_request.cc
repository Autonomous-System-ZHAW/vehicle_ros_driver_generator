#include <pix_nina_driver/remote_application_toggle_request.hpp>

int32_t RemoteApplicationToggleRequest::ID = 0x0;

// public
RemoteApplicationToggleRequest::RemoteApplicationToggleRequest() { Reset(); }

void RemoteApplicationToggleRequest::UpdateData(bool app_toggle_req__speed_limit4kmh, bool app_toggle_req__send_c_a_n_dbg_messages, bool app_toggle_req__pwr_assisted_braking) {
  set_p_app_toggle_req__speed_limit4kmh(app_toggle_req__speed_limit4kmh);
  set_p_app_toggle_req__send_c_a_n_dbg_messages(app_toggle_req__send_c_a_n_dbg_messages);
  set_p_app_toggle_req__pwr_assisted_braking(app_toggle_req__pwr_assisted_braking);
}

void RemoteApplicationToggleRequest::Reset() {
  // TODO(All) :  you should check this manually
  for(uint8_t i=0;i<8;i++)
  {
    data[i] = 0;
  }
}

uint8_t * RemoteApplicationToggleRequest::get_data()
{
  return data;
}



// config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'app_toggle_req__speed_limit4kmh', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void RemoteApplicationToggleRequest::set_p_app_toggle_req__speed_limit4kmh(bool app_toggle_req__speed_limit4kmh) {
  int x = app_toggle_req__speed_limit4kmh;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 1);
  data[0] += to_set.return_byte_t();
  
}

// config detail: {'bit': 11, 'is_signed_var': False, 'len': 1, 'name': 'app_toggle_req__send_c_a_n_dbg_messages', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void RemoteApplicationToggleRequest::set_p_app_toggle_req__send_c_a_n_dbg_messages(bool app_toggle_req__send_c_a_n_dbg_messages) {
  int x = app_toggle_req__send_c_a_n_dbg_messages;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 3, 1);
  data[1] += to_set.return_byte_t();
  
}

// config detail: {'bit': 1, 'is_signed_var': False, 'len': 1, 'name': 'app_toggle_req__pwr_assisted_braking', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void RemoteApplicationToggleRequest::set_p_app_toggle_req__pwr_assisted_braking(bool app_toggle_req__pwr_assisted_braking) {
  int x = app_toggle_req__pwr_assisted_braking;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 1, 1);
  data[0] += to_set.return_byte_t();
  
}


