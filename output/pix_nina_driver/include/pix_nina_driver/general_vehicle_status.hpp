#pragma once

#include <pix_nina_driver/Byte.hpp>
#include <iostream>

class GeneralVehicleStatus {
public:
    static const uint32_t ID = 0x2;
    GeneralVehicleStatus();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    bool app_status__speed_limit4kmh_;
    bool app_status__send_c_a_n_dbg_messages_;
    bool app_status__pwr_assisted_braking_;
    bool signal12_switch_;
    bool request_zero_throttle_;
    bool request_brake_;
    bool signal_brights_on_;
    bool signal_right_turn_;
    bool signal_left_turn_;
    bool signal_horn_;
    bool signal_hazard_lights_;
    bool signal_fog_lights_;
    bool signal_direction_reverse_;
    bool global_man_sig_mag_brake_;
    bool global_curtis_sig_mag_brake_;
    bool signal_brake_switch_;
    bool signal_seat_switch_;
    bool e_stop_status_;
    int selected_application_;
    int selected_op_mode_;
    int active_op_mode_;
    bool button_blue_;
    bool button_yellow_;
    bool button_green_;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 1, 'name': 'app_status__speed_limit4kmh', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool appstatusspeedlimit4kmh();

  // config detail: {'bit': 27, 'is_signed_var': False, 'len': 1, 'name': 'app_status__send_c_a_n_dbg_messages', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool appstatussendcandbgmessages();

  // config detail: {'bit': 17, 'is_signed_var': False, 'len': 1, 'name': 'app_status__pwr_assisted_braking', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool appstatuspwrassistedbraking();

  // config detail: {'bit': 39, 'is_signed_var': False, 'len': 1, 'name': 'signal12_switch', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool signal12switch();

  // config detail: {'bit': 56, 'is_signed_var': False, 'len': 1, 'name': 'request_zero_throttle', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool requestzerothrottle();

  // config detail: {'bit': 57, 'is_signed_var': False, 'len': 1, 'name': 'request_brake', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool requestbrake();

  // config detail: {'bit': 35, 'is_signed_var': False, 'len': 1, 'name': 'signal_brights_on', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool signalbrightson();

  // config detail: {'bit': 32, 'is_signed_var': False, 'len': 1, 'name': 'signal_right_turn', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool signalrightturn();

  // config detail: {'bit': 33, 'is_signed_var': False, 'len': 1, 'name': 'signal_left_turn', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool signalleftturn();

  // config detail: {'bit': 38, 'is_signed_var': False, 'len': 1, 'name': 'signal_horn', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool signalhorn();

  // config detail: {'bit': 37, 'is_signed_var': False, 'len': 1, 'name': 'signal_hazard_lights', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool signalhazardlights();

  // config detail: {'bit': 36, 'is_signed_var': False, 'len': 1, 'name': 'signal_fog_lights', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool signalfoglights();

  // config detail: {'bit': 34, 'is_signed_var': False, 'len': 1, 'name': 'signal_direction_reverse', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool signaldirectionreverse();

  // config detail: {'bit': 30, 'is_signed_var': False, 'len': 1, 'name': 'global_man_sig_mag_brake', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool globalmansigmagbrake();

  // config detail: {'bit': 31, 'is_signed_var': False, 'len': 1, 'name': 'global_curtis_sig_mag_brake', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool globalcurtissigmagbrake();

  // config detail: {'bit': 29, 'is_signed_var': False, 'len': 1, 'name': 'signal_brake_switch', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool signalbrakeswitch();

  // config detail: {'bit': 28, 'is_signed_var': False, 'len': 1, 'name': 'signal_seat_switch', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool signalseatswitch();

  // config detail: {'bit': 3, 'is_signed_var': False, 'len': 1, 'name': 'e_stop_status', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool estopstatus();

  // config detail: {'bit': 4, 'is_signed_var': False, 'len': 4, 'name': 'selected_application', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int selectedapplication();

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 4, 'name': 'selected_op_mode', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int selectedopmode();

  // config detail: {'bit': 12, 'is_signed_var': False, 'len': 4, 'name': 'active_op_mode', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int activeopmode();

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'button_blue', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool buttonblue();

  // config detail: {'bit': 1, 'is_signed_var': False, 'len': 1, 'name': 'button_yellow', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool buttonyellow();

  // config detail: {'bit': 2, 'is_signed_var': False, 'len': 1, 'name': 'button_green', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool buttongreen();
};



