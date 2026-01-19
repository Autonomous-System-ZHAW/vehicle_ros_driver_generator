#pragma once
#include <pix_nina_driver/Byte.hpp>

class RemoteIndicatorRequest {
public:
	static  int32_t ID;

	RemoteIndicatorRequest();

  	void UpdateData(int remote_status_light_blink_rate_req, int remote_status_light_color_req, bool remote_horn_req, bool remote_light_brake_req, bool remote_blink_left_req, bool remote_blink_right_req, bool remote_brights_on_req, bool remote_light_reverse_req, bool remote_lights_turn_left_req, bool remote_lights_turn_right_req);

  	void Reset();
  
  	uint8_t *get_data();


private:
	
  // config detail: {'bit': 10, 'is_signed_var': False, 'len': 2, 'name': 'remote_status_light_blink_rate_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_remote_status_light_blink_rate_req(int remote_status_light_blink_rate_req);

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 2, 'name': 'remote_status_light_color_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_remote_status_light_color_req(int remote_status_light_color_req);

  // config detail: {'bit': 7, 'is_signed_var': False, 'len': 1, 'name': 'remote_horn_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_remote_horn_req(bool remote_horn_req);

  // config detail: {'bit': 6, 'is_signed_var': False, 'len': 1, 'name': 'remote_light_brake_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_remote_light_brake_req(bool remote_light_brake_req);

  // config detail: {'bit': 5, 'is_signed_var': False, 'len': 1, 'name': 'remote_blink_left_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_remote_blink_left_req(bool remote_blink_left_req);

  // config detail: {'bit': 4, 'is_signed_var': False, 'len': 1, 'name': 'remote_blink_right_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_remote_blink_right_req(bool remote_blink_right_req);

  // config detail: {'bit': 3, 'is_signed_var': False, 'len': 1, 'name': 'remote_brights_on_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_remote_brights_on_req(bool remote_brights_on_req);

  // config detail: {'bit': 2, 'is_signed_var': False, 'len': 1, 'name': 'remote_light_reverse_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_remote_light_reverse_req(bool remote_light_reverse_req);

  // config detail: {'bit': 1, 'is_signed_var': False, 'len': 1, 'name': 'remote_lights_turn_left_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_remote_lights_turn_left_req(bool remote_lights_turn_left_req);

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'remote_lights_turn_right_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_remote_lights_turn_right_req(bool remote_lights_turn_right_req);

private:
	uint8_t data[8];
};



