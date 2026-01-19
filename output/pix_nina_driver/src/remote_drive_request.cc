#include <pix_nina_driver/remote_drive_request.hpp>

int32_t RemoteDriveRequest::ID = 0x11;

// public
RemoteDriveRequest::RemoteDriveRequest() { Reset(); }

void RemoteDriveRequest::UpdateData(double remote_steering_angle_req, double remote_velocity_req) {
  set_p_remote_steering_angle_req(remote_steering_angle_req);
  set_p_remote_velocity_req(remote_velocity_req);
}

void RemoteDriveRequest::Reset() {
  // TODO(All) :  you should check this manually
  for(uint8_t i=0;i<8;i++)
  {
    data[i] = 0;
  }
}

uint8_t * RemoteDriveRequest::get_data()
{
  return data;
}



// config detail: {'bit': 16, 'is_signed_var': True, 'len': 16, 'name': 'remote_steering_angle_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-327.68|327.67]', 'physical_unit': '%', 'precision': 0.01, 'type': 'double'}
void RemoteDriveRequest::set_p_remote_steering_angle_req(double remote_steering_angle_req) {
  // remote_steering_angle_req = ProtocolData::BoundedValue(-327.68, 327.67, remote_steering_angle_req);
  int x = remote_steering_angle_req / 0.010000;
  uint8_t a = 0;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(a);
  to_set0.set_value(t, 0, 8);
  data[2] += to_set0.return_byte_t();
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(a);
  to_set1.set_value(t, 0, 8);
  data[3] += to_set1.return_byte_t();
}

// config detail: {'bit': 0, 'is_signed_var': True, 'len': 16, 'name': 'remote_velocity_req', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-32.768|32.767]', 'physical_unit': 'm/s', 'precision': 0.001, 'type': 'double'}
void RemoteDriveRequest::set_p_remote_velocity_req(double remote_velocity_req) {
  // remote_velocity_req = ProtocolData::BoundedValue(-32.768, 32.767, remote_velocity_req);
  int x = remote_velocity_req / 0.001000;
  uint8_t a = 0;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(a);
  to_set0.set_value(t, 0, 8);
  data[0] += to_set0.return_byte_t();
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(a);
  to_set1.set_value(t, 0, 8);
  data[1] += to_set1.return_byte_t();
}


