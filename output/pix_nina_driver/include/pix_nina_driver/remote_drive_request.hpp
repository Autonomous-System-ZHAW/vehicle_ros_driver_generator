#pragma once
#include <pix_nina_driver/Byte.hpp>

class RemoteDriveRequest {
public:
	static  int32_t ID;

	RemoteDriveRequest();

  	void UpdateData(double remote_steering_angle_req, double remote_velocity_req);

  	void Reset();
  
  	uint8_t *get_data();


private:
	
  // config detail: {'bit': 16, 'is_signed_var': True, 'len': 16, 'name': 'remote_steering_angle_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-327.68|327.67]', 'physical_unit': '%', 'precision': 0.01, 'type': 'double'}
  void set_p_remote_steering_angle_req(double remote_steering_angle_req);

  // config detail: {'bit': 0, 'is_signed_var': True, 'len': 16, 'name': 'remote_velocity_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-32.768|32.767]', 'physical_unit': 'm/s', 'precision': 0.001, 'type': 'double'}
  void set_p_remote_velocity_req(double remote_velocity_req);

private:
	uint8_t data[8];
};



