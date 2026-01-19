#pragma once
#include <pix_nina_driver/Byte.hpp>

class RemoteApplicationToggleRequest {
public:
	static  int32_t ID;

	RemoteApplicationToggleRequest();

  	void UpdateData(bool app_toggle_req__speed_limit4kmh, bool app_toggle_req__send_c_a_n_dbg_messages, bool app_toggle_req__pwr_assisted_braking);

  	void Reset();
  
  	uint8_t *get_data();


private:
	
  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'app_toggle_req__speed_limit4kmh', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_app_toggle_req__speed_limit4kmh(bool app_toggle_req__speed_limit4kmh);

  // config detail: {'bit': 11, 'is_signed_var': False, 'len': 1, 'name': 'app_toggle_req__send_c_a_n_dbg_messages', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_app_toggle_req__send_c_a_n_dbg_messages(bool app_toggle_req__send_c_a_n_dbg_messages);

  // config detail: {'bit': 1, 'is_signed_var': False, 'len': 1, 'name': 'app_toggle_req__pwr_assisted_braking', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_app_toggle_req__pwr_assisted_braking(bool app_toggle_req__pwr_assisted_braking);

private:
	uint8_t data[8];
};



