#include <pix_nina_driver/general_vehicle_status.hpp>


GeneralVehicleStatus::GeneralVehicleStatus() {}

void GeneralVehicleStatus::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void GeneralVehicleStatus::Parse() {
  app_status__speed_limit4kmh_ = appstatusspeedlimit4kmh();
  app_status__send_c_a_n_dbg_messages_ = appstatussendcandbgmessages();
  app_status__pwr_assisted_braking_ = appstatuspwrassistedbraking();
  signal12_switch_ = signal12switch();
  request_zero_throttle_ = requestzerothrottle();
  request_brake_ = requestbrake();
  signal_brights_on_ = signalbrightson();
  signal_right_turn_ = signalrightturn();
  signal_left_turn_ = signalleftturn();
  signal_horn_ = signalhorn();
  signal_hazard_lights_ = signalhazardlights();
  signal_fog_lights_ = signalfoglights();
  signal_direction_reverse_ = signaldirectionreverse();
  global_man_sig_mag_brake_ = globalmansigmagbrake();
  global_curtis_sig_mag_brake_ = globalcurtissigmagbrake();
  signal_brake_switch_ = signalbrakeswitch();
  signal_seat_switch_ = signalseatswitch();
  e_stop_status_ = estopstatus();
  selected_application_ = selectedapplication();
  selected_op_mode_ = selectedopmode();
  active_op_mode_ = activeopmode();
  button_blue_ = buttonblue();
  button_yellow_ = buttonyellow();
  button_green_ = buttongreen();
}


// config detail: {'bit': 16, 'is_signed_var': False, 'len': 1, 'name': 'app_status__speed_limit4kmh', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool GeneralVehicleStatus::appstatusspeedlimit4kmh() {
  Byte t0(*(bytes + 2));
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 27, 'is_signed_var': False, 'len': 1, 'name': 'app_status__send_c_a_n_dbg_messages', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool GeneralVehicleStatus::appstatussendcandbgmessages() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 17, 'is_signed_var': False, 'len': 1, 'name': 'app_status__pwr_assisted_braking', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool GeneralVehicleStatus::appstatuspwrassistedbraking() {
  Byte t0(*(bytes + 2));
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 39, 'is_signed_var': False, 'len': 1, 'name': 'signal12_switch', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool GeneralVehicleStatus::signal12switch() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 56, 'is_signed_var': False, 'len': 1, 'name': 'request_zero_throttle', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool GeneralVehicleStatus::requestzerothrottle() {
  Byte t0(*(bytes + 7));
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 57, 'is_signed_var': False, 'len': 1, 'name': 'request_brake', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool GeneralVehicleStatus::requestbrake() {
  Byte t0(*(bytes + 7));
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 35, 'is_signed_var': False, 'len': 1, 'name': 'signal_brights_on', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool GeneralVehicleStatus::signalbrightson() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 32, 'is_signed_var': False, 'len': 1, 'name': 'signal_right_turn', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool GeneralVehicleStatus::signalrightturn() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 33, 'is_signed_var': False, 'len': 1, 'name': 'signal_left_turn', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool GeneralVehicleStatus::signalleftturn() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 38, 'is_signed_var': False, 'len': 1, 'name': 'signal_horn', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool GeneralVehicleStatus::signalhorn() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 37, 'is_signed_var': False, 'len': 1, 'name': 'signal_hazard_lights', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool GeneralVehicleStatus::signalhazardlights() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 36, 'is_signed_var': False, 'len': 1, 'name': 'signal_fog_lights', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool GeneralVehicleStatus::signalfoglights() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 34, 'is_signed_var': False, 'len': 1, 'name': 'signal_direction_reverse', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool GeneralVehicleStatus::signaldirectionreverse() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 30, 'is_signed_var': False, 'len': 1, 'name': 'global_man_sig_mag_brake', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool GeneralVehicleStatus::globalmansigmagbrake() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 31, 'is_signed_var': False, 'len': 1, 'name': 'global_curtis_sig_mag_brake', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool GeneralVehicleStatus::globalcurtissigmagbrake() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 29, 'is_signed_var': False, 'len': 1, 'name': 'signal_brake_switch', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool GeneralVehicleStatus::signalbrakeswitch() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 28, 'is_signed_var': False, 'len': 1, 'name': 'signal_seat_switch', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool GeneralVehicleStatus::signalseatswitch() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 3, 'is_signed_var': False, 'len': 1, 'name': 'e_stop_status', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool GeneralVehicleStatus::estopstatus() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 4, 'is_signed_var': False, 'len': 4, 'name': 'selected_application', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int GeneralVehicleStatus::selectedapplication() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

// config detail: {'bit': 8, 'is_signed_var': False, 'len': 4, 'name': 'selected_op_mode', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int GeneralVehicleStatus::selectedopmode() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'bit': 12, 'is_signed_var': False, 'len': 4, 'name': 'active_op_mode', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int GeneralVehicleStatus::activeopmode() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'button_blue', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool GeneralVehicleStatus::buttonblue() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 1, 'is_signed_var': False, 'len': 1, 'name': 'button_yellow', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool GeneralVehicleStatus::buttonyellow() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 2, 'is_signed_var': False, 'len': 1, 'name': 'button_green', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool GeneralVehicleStatus::buttongreen() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

