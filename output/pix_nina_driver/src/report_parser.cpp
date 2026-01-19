#include <pix_nina_driver/report_parser.hpp>

namespace pix_nina_driver
{
namespace report_parser
{
ReportParser::ReportParser() : Node("report_parser")
{
  // ros params
  param_.base_frame_id = declare_parameter("base_frame_id", "base_link");
  param_.report_timeout_ms = declare_parameter("report_timeout_ms", 1000);
  param_.loop_rate = declare_parameter("loop_rate", 50.0);

  // // initialize msg received time
  /** example
  brake_command_received_time_ = this->now();
  **/
  general_vehicle_status_received_time_ = this->now();
steering_and_speed_received_time_ = this->now();
u_s_sensor_front_received_time_ = this->now();
u_s_sensor_rear_received_time_ = this->now();
battery_status_received_time_ = this->now();
raw_signal_throttle_received_time_ = this->now();
raw_signal_break_received_time_ = this->now();
raw_signal_steering_force_received_time_ = this->now();
raw_signal_steering_position_received_time_ = this->now();
scaled_signals_received_time_ = this->now();
raw_signals_steering_and_speed_received_time_ = this->now();
temperatures_received_time_ = this->now();
v_e_c_t_o_r___i_n_d_e_p_e_n_d_e_n_t__s_i_g__m_s_g_received_time_ = this->now();


  is_publish_ = true;

  using std::placeholders::_1;

  /* subscriber */
  {
    // from pix driver autoware interface
    can_frame_sub_ = create_subscription<can_msgs::msg::Frame>(
      "input/can_rx", 1, std::bind(&ReportParser::callbackCan, this, _1));
    // is publish
    is_publish_sub_ = create_subscription<std_msgs::msg::Bool>(
      "input/is_publish", 1, std::bind(&ReportParser::callbackIsPublish, this, _1));
  }

  /* publisher */
  {
    /** example
    brake_sta_fb_pub_ =
      create_publisher<V2aBrakeStaFb>("/pix_hooke/v2a_brakestafb", rclcpp::QoS{1});
    **/
    general_vehicle_status_pub_ = create_publisher<pix_nina_driver_msgs::msg::GeneralVehicleStatus>("/pix_nina/general_vehicle_status", rclcpp::QoS{1});
steering_and_speed_pub_ = create_publisher<pix_nina_driver_msgs::msg::SteeringAndSpeed>("/pix_nina/steering_and_speed", rclcpp::QoS{1});
u_s_sensor_front_pub_ = create_publisher<pix_nina_driver_msgs::msg::USSensorFront>("/pix_nina/u_s_sensor_front", rclcpp::QoS{1});
u_s_sensor_rear_pub_ = create_publisher<pix_nina_driver_msgs::msg::USSensorRear>("/pix_nina/u_s_sensor_rear", rclcpp::QoS{1});
battery_status_pub_ = create_publisher<pix_nina_driver_msgs::msg::BatteryStatus>("/pix_nina/battery_status", rclcpp::QoS{1});
raw_signal_throttle_pub_ = create_publisher<pix_nina_driver_msgs::msg::RawSignalThrottle>("/pix_nina/raw_signal_throttle", rclcpp::QoS{1});
raw_signal_break_pub_ = create_publisher<pix_nina_driver_msgs::msg::RawSignalBreak>("/pix_nina/raw_signal_break", rclcpp::QoS{1});
raw_signal_steering_force_pub_ = create_publisher<pix_nina_driver_msgs::msg::RawSignalSteeringForce>("/pix_nina/raw_signal_steering_force", rclcpp::QoS{1});
raw_signal_steering_position_pub_ = create_publisher<pix_nina_driver_msgs::msg::RawSignalSteeringPosition>("/pix_nina/raw_signal_steering_position", rclcpp::QoS{1});
scaled_signals_pub_ = create_publisher<pix_nina_driver_msgs::msg::ScaledSignals>("/pix_nina/scaled_signals", rclcpp::QoS{1});
raw_signals_steering_and_speed_pub_ = create_publisher<pix_nina_driver_msgs::msg::RawSignalsSteeringAndSpeed>("/pix_nina/raw_signals_steering_and_speed", rclcpp::QoS{1});
temperatures_pub_ = create_publisher<pix_nina_driver_msgs::msg::Temperatures>("/pix_nina/temperatures", rclcpp::QoS{1});
v_e_c_t_o_r___i_n_d_e_p_e_n_d_e_n_t__s_i_g__m_s_g_pub_ = create_publisher<pix_nina_driver_msgs::msg::VECTORINDEPENDENTSIGMSG>("/pix_nina/v_e_c_t_o_r___i_n_d_e_p_e_n_d_e_n_t__s_i_g__m_s_g", rclcpp::QoS{1});
 
  }
  {
    // timer
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Rate(param_.loop_rate).period(),
      std::bind(&ReportParser::timerCallback, this));
  }
}

// calback is publish
void ReportParser::callbackIsPublish(const std_msgs::msg::Bool::ConstSharedPtr & msg)
{
  is_publish_ = msg->data;
}

// callback can
void ReportParser::callbackCan(const can_msgs::msg::Frame::ConstSharedPtr & msg)
{
  std_msgs::msg::Header header;
  header.frame_id = param_.base_frame_id;
  header.stamp = msg->header.stamp;

  /** example
  V2aBrakeStaFb brake_sta_fb_msg;
  **/
  pix_nina_driver_msgs::msg::GeneralVehicleStatus general_vehicle_status_msg;
pix_nina_driver_msgs::msg::SteeringAndSpeed steering_and_speed_msg;
pix_nina_driver_msgs::msg::USSensorFront u_s_sensor_front_msg;
pix_nina_driver_msgs::msg::USSensorRear u_s_sensor_rear_msg;
pix_nina_driver_msgs::msg::BatteryStatus battery_status_msg;
pix_nina_driver_msgs::msg::RawSignalThrottle raw_signal_throttle_msg;
pix_nina_driver_msgs::msg::RawSignalBreak raw_signal_break_msg;
pix_nina_driver_msgs::msg::RawSignalSteeringForce raw_signal_steering_force_msg;
pix_nina_driver_msgs::msg::RawSignalSteeringPosition raw_signal_steering_position_msg;
pix_nina_driver_msgs::msg::ScaledSignals scaled_signals_msg;
pix_nina_driver_msgs::msg::RawSignalsSteeringAndSpeed raw_signals_steering_and_speed_msg;
pix_nina_driver_msgs::msg::Temperatures temperatures_msg;
pix_nina_driver_msgs::msg::VECTORINDEPENDENTSIGMSG v_e_c_t_o_r___i_n_d_e_p_e_n_d_e_n_t__s_i_g__m_s_g_msg;


  uint8_t byte_temp[8];
  switch (msg->id)
  {
  /** example
  case V2adrivestafb530::ID:
    drive_sta_fb_received_time_ = this->now();
    
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    v2a_drivestafb_530_entity_.update_bytes(byte_temp);
    v2a_drivestafb_530_entity_.Parse();

    drive_sta_fb_msg.header = header;
    drive_sta_fb_msg.vcu_chassis_driver_en_sta =
      v2a_drivestafb_530_entity_.vcu_chassisdriverensta;
    drive_sta_fb_msg.vcu_chassis_diver_slopover =
      v2a_drivestafb_530_entity_.vcu_chassisdiverslopover;
    drive_sta_fb_msg.vcu_chassis_driver_mode_sta =
      v2a_drivestafb_530_entity_.vcu_chassisdrivermodesta;
    drive_sta_fb_msg.vcu_chassis_gear_fb = v2a_drivestafb_530_entity_.vcu_chassisgearfb;
    drive_sta_fb_msg.vcu_chassis_speed_fb = v2a_drivestafb_530_entity_.vcu_chassisspeedfb;
    drive_sta_fb_msg.vcu_chassis_throttle_padl_fb =
      v2a_drivestafb_530_entity_.vcu_chassisthrottlepaldfb;
    drive_sta_fb_msg.vcu_chassis_accceleration_fb =
      v2a_drivestafb_530_entity_.vcu_chassisaccelerationfb;
    drive_sta_fb_ptr_ = std::make_shared<V2aDriveStaFb>(drive_sta_fb_msg);
    break;
  **/
  

    case GeneralVehicleStatus::ID:
    general_vehicle_status_received_time_ = this->now();
    
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    general_vehicle_status_entity_.update_bytes(byte_temp);
    general_vehicle_status_entity_.Parse();

    general_vehicle_status_msg.header = header;
    general_vehicle_status_msg.app_status__speed_limit4kmh = general_vehicle_status_entity_.app_status__speed_limit4kmh_;
general_vehicle_status_msg.app_status__send_c_a_n_dbg_messages = general_vehicle_status_entity_.app_status__send_c_a_n_dbg_messages_;
general_vehicle_status_msg.app_status__pwr_assisted_braking = general_vehicle_status_entity_.app_status__pwr_assisted_braking_;
general_vehicle_status_msg.signal12_switch = general_vehicle_status_entity_.signal12_switch_;
general_vehicle_status_msg.request_zero_throttle = general_vehicle_status_entity_.request_zero_throttle_;
general_vehicle_status_msg.request_brake = general_vehicle_status_entity_.request_brake_;
general_vehicle_status_msg.signal_brights_on = general_vehicle_status_entity_.signal_brights_on_;
general_vehicle_status_msg.signal_right_turn = general_vehicle_status_entity_.signal_right_turn_;
general_vehicle_status_msg.signal_left_turn = general_vehicle_status_entity_.signal_left_turn_;
general_vehicle_status_msg.signal_horn = general_vehicle_status_entity_.signal_horn_;
general_vehicle_status_msg.signal_hazard_lights = general_vehicle_status_entity_.signal_hazard_lights_;
general_vehicle_status_msg.signal_fog_lights = general_vehicle_status_entity_.signal_fog_lights_;
general_vehicle_status_msg.signal_direction_reverse = general_vehicle_status_entity_.signal_direction_reverse_;
general_vehicle_status_msg.global_man_sig_mag_brake = general_vehicle_status_entity_.global_man_sig_mag_brake_;
general_vehicle_status_msg.global_curtis_sig_mag_brake = general_vehicle_status_entity_.global_curtis_sig_mag_brake_;
general_vehicle_status_msg.signal_brake_switch = general_vehicle_status_entity_.signal_brake_switch_;
general_vehicle_status_msg.signal_seat_switch = general_vehicle_status_entity_.signal_seat_switch_;
general_vehicle_status_msg.e_stop_status = general_vehicle_status_entity_.e_stop_status_;
general_vehicle_status_msg.selected_application = general_vehicle_status_entity_.selected_application_;
general_vehicle_status_msg.selected_op_mode = general_vehicle_status_entity_.selected_op_mode_;
general_vehicle_status_msg.active_op_mode = general_vehicle_status_entity_.active_op_mode_;
general_vehicle_status_msg.button_blue = general_vehicle_status_entity_.button_blue_;
general_vehicle_status_msg.button_yellow = general_vehicle_status_entity_.button_yellow_;
general_vehicle_status_msg.button_green = general_vehicle_status_entity_.button_green_;

    general_vehicle_status_ptr_ = std::make_shared<pix_nina_driver_msgs::msg::GeneralVehicleStatus>(general_vehicle_status_msg);
    break;
    

    case SteeringAndSpeed::ID:
    steering_and_speed_received_time_ = this->now();
    
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    steering_and_speed_entity_.update_bytes(byte_temp);
    steering_and_speed_entity_.Parse();

    steering_and_speed_msg.header = header;
    steering_and_speed_msg.vehicle_velocity_requested = steering_and_speed_entity_.vehicle_velocity_requested_;
steering_and_speed_msg.steering_position_requested = steering_and_speed_entity_.steering_position_requested_;
steering_and_speed_msg.vehicle_velocity_measured = steering_and_speed_entity_.vehicle_velocity_measured_;
steering_and_speed_msg.steering_position_measured = steering_and_speed_entity_.steering_position_measured_;

    steering_and_speed_ptr_ = std::make_shared<pix_nina_driver_msgs::msg::SteeringAndSpeed>(steering_and_speed_msg);
    break;
    

    case USSensorFront::ID:
    u_s_sensor_front_received_time_ = this->now();
    
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    u_s_sensor_front_entity_.update_bytes(byte_temp);
    u_s_sensor_front_entity_.Parse();

    u_s_sensor_front_msg.header = header;
    u_s_sensor_front_msg.u_s_sensor4 = u_s_sensor_front_entity_.u_s_sensor4_;
u_s_sensor_front_msg.u_s_sensor3 = u_s_sensor_front_entity_.u_s_sensor3_;
u_s_sensor_front_msg.u_s_sensor2 = u_s_sensor_front_entity_.u_s_sensor2_;
u_s_sensor_front_msg.u_s_sensor1 = u_s_sensor_front_entity_.u_s_sensor1_;

    u_s_sensor_front_ptr_ = std::make_shared<pix_nina_driver_msgs::msg::USSensorFront>(u_s_sensor_front_msg);
    break;
    

    case USSensorRear::ID:
    u_s_sensor_rear_received_time_ = this->now();
    
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    u_s_sensor_rear_entity_.update_bytes(byte_temp);
    u_s_sensor_rear_entity_.Parse();

    u_s_sensor_rear_msg.header = header;
    u_s_sensor_rear_msg.u_s_sensor8 = u_s_sensor_rear_entity_.u_s_sensor8_;
u_s_sensor_rear_msg.u_s_sensor7 = u_s_sensor_rear_entity_.u_s_sensor7_;
u_s_sensor_rear_msg.u_s_sensor6 = u_s_sensor_rear_entity_.u_s_sensor6_;
u_s_sensor_rear_msg.u_s_sensor5 = u_s_sensor_rear_entity_.u_s_sensor5_;

    u_s_sensor_rear_ptr_ = std::make_shared<pix_nina_driver_msgs::msg::USSensorRear>(u_s_sensor_rear_msg);
    break;
    

    case BatteryStatus::ID:
    battery_status_received_time_ = this->now();
    
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    battery_status_entity_.update_bytes(byte_temp);
    battery_status_entity_.Parse();

    battery_status_msg.header = header;
    battery_status_msg.battery_discharge_percent = battery_status_entity_.battery_discharge_percent_;
battery_status_msg.battery_current = battery_status_entity_.battery_current_;
battery_status_msg.battery_voltage = battery_status_entity_.battery_voltage_;

    battery_status_ptr_ = std::make_shared<pix_nina_driver_msgs::msg::BatteryStatus>(battery_status_msg);
    break;
    

    case RawSignalThrottle::ID:
    raw_signal_throttle_received_time_ = this->now();
    
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    raw_signal_throttle_entity_.update_bytes(byte_temp);
    raw_signal_throttle_entity_.Parse();

    raw_signal_throttle_msg.header = header;
    raw_signal_throttle_msg.uint32_signal_throttle_b = raw_signal_throttle_entity_.uint32_signal_throttle_b_;
raw_signal_throttle_msg.uint32_signal_throttle_a = raw_signal_throttle_entity_.uint32_signal_throttle_a_;

    raw_signal_throttle_ptr_ = std::make_shared<pix_nina_driver_msgs::msg::RawSignalThrottle>(raw_signal_throttle_msg);
    break;
    

    case RawSignalBreak::ID:
    raw_signal_break_received_time_ = this->now();
    
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    raw_signal_break_entity_.update_bytes(byte_temp);
    raw_signal_break_entity_.Parse();

    raw_signal_break_msg.header = header;
    raw_signal_break_msg.uint32_signal_brake_b = raw_signal_break_entity_.uint32_signal_brake_b_;
raw_signal_break_msg.uint32_signal_brake_a = raw_signal_break_entity_.uint32_signal_brake_a_;

    raw_signal_break_ptr_ = std::make_shared<pix_nina_driver_msgs::msg::RawSignalBreak>(raw_signal_break_msg);
    break;
    

    case RawSignalSteeringForce::ID:
    raw_signal_steering_force_received_time_ = this->now();
    
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    raw_signal_steering_force_entity_.update_bytes(byte_temp);
    raw_signal_steering_force_entity_.Parse();

    raw_signal_steering_force_msg.header = header;
    raw_signal_steering_force_msg.uint32_signal_steering_force_b = raw_signal_steering_force_entity_.uint32_signal_steering_force_b_;
raw_signal_steering_force_msg.uint32_signal_steering_force_a = raw_signal_steering_force_entity_.uint32_signal_steering_force_a_;

    raw_signal_steering_force_ptr_ = std::make_shared<pix_nina_driver_msgs::msg::RawSignalSteeringForce>(raw_signal_steering_force_msg);
    break;
    

    case RawSignalSteeringPosition::ID:
    raw_signal_steering_position_received_time_ = this->now();
    
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    raw_signal_steering_position_entity_.update_bytes(byte_temp);
    raw_signal_steering_position_entity_.Parse();

    raw_signal_steering_position_msg.header = header;
    raw_signal_steering_position_msg.uint16_encoder_raw_value_b = raw_signal_steering_position_entity_.uint16_encoder_raw_value_b_;
raw_signal_steering_position_msg.uint16_encoder_raw_value_a = raw_signal_steering_position_entity_.uint16_encoder_raw_value_a_;

    raw_signal_steering_position_ptr_ = std::make_shared<pix_nina_driver_msgs::msg::RawSignalSteeringPosition>(raw_signal_steering_position_msg);
    break;
    

    case ScaledSignals::ID:
    scaled_signals_received_time_ = this->now();
    
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    scaled_signals_entity_.update_bytes(byte_temp);
    scaled_signals_entity_.Parse();

    scaled_signals_msg.header = header;
    scaled_signals_msg.throttle_signal = scaled_signals_entity_.throttle_signal_;
scaled_signals_msg.steering_motor_speed_cmd = scaled_signals_entity_.steering_motor_speed_cmd_;
scaled_signals_msg.curtis_speed_cmd = scaled_signals_entity_.curtis_speed_cmd_;
scaled_signals_msg.steering_torque_signal = scaled_signals_entity_.steering_torque_signal_;
scaled_signals_msg.steering_velocity_signal = scaled_signals_entity_.steering_velocity_signal_;
scaled_signals_msg.brake_signal = scaled_signals_entity_.brake_signal_;

    scaled_signals_ptr_ = std::make_shared<pix_nina_driver_msgs::msg::ScaledSignals>(scaled_signals_msg);
    break;
    

    case RawSignalsSteeringAndSpeed::ID:
    raw_signals_steering_and_speed_received_time_ = this->now();
    
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    raw_signals_steering_and_speed_entity_.update_bytes(byte_temp);
    raw_signals_steering_and_speed_entity_.Parse();

    raw_signals_steering_and_speed_msg.header = header;
    raw_signals_steering_and_speed_msg.uint8_vehicle_speed = raw_signals_steering_and_speed_entity_.uint8_vehicle_speed_;
raw_signals_steering_and_speed_msg.uint16_poti_throttle_cmd = raw_signals_steering_and_speed_entity_.uint16_poti_throttle_cmd_;
raw_signals_steering_and_speed_msg.int16_steering_velocity_cmd = raw_signals_steering_and_speed_entity_.int16_steering_velocity_cmd_;
raw_signals_steering_and_speed_msg.int16_steering_velocity = raw_signals_steering_and_speed_entity_.int16_steering_velocity_;

    raw_signals_steering_and_speed_ptr_ = std::make_shared<pix_nina_driver_msgs::msg::RawSignalsSteeringAndSpeed>(raw_signals_steering_and_speed_msg);
    break;
    

    case Temperatures::ID:
    temperatures_received_time_ = this->now();
    
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    temperatures_entity_.update_bytes(byte_temp);
    temperatures_entity_.Parse();

    temperatures_msg.header = header;
    temperatures_msg.steering_motor_inverter_temp = temperatures_entity_.steering_motor_inverter_temp_;
temperatures_msg.steering_motor_temp = temperatures_entity_.steering_motor_temp_;
temperatures_msg.curtis_controller_temp = temperatures_entity_.curtis_controller_temp_;
temperatures_msg.curtis_motor_temp = temperatures_entity_.curtis_motor_temp_;

    temperatures_ptr_ = std::make_shared<pix_nina_driver_msgs::msg::Temperatures>(temperatures_msg);
    break;
    

    case VECTORINDEPENDENTSIGMSG::ID:
    v_e_c_t_o_r___i_n_d_e_p_e_n_d_e_n_t__s_i_g__m_s_g_received_time_ = this->now();
    
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    v_e_c_t_o_r___i_n_d_e_p_e_n_d_e_n_t__s_i_g__m_s_g_entity_.update_bytes(byte_temp);
    v_e_c_t_o_r___i_n_d_e_p_e_n_d_e_n_t__s_i_g__m_s_g_entity_.Parse();

    v_e_c_t_o_r___i_n_d_e_p_e_n_d_e_n_t__s_i_g__m_s_g_msg.header = header;
    v_e_c_t_o_r___i_n_d_e_p_e_n_d_e_n_t__s_i_g__m_s_g_msg.enabled_applications = v_e_c_t_o_r___i_n_d_e_p_e_n_d_e_n_t__s_i_g__m_s_g_entity_.enabled_applications_;

    v_e_c_t_o_r___i_n_d_e_p_e_n_d_e_n_t__s_i_g__m_s_g_ptr_ = std::make_shared<pix_nina_driver_msgs::msg::VECTORINDEPENDENTSIGMSG>(v_e_c_t_o_r___i_n_d_e_p_e_n_d_e_n_t__s_i_g__m_s_g_msg);
    break;
    

  default:
    break;
  }
}

void ReportParser::timerCallback()
{
  if (!is_publish_) return;

  const rclcpp::Time current_time = this->now();
  
  /** example
  // drive sta fb report
  const double drive_sta_fb_report_delta_time_ms =
    (current_time - drive_sta_fb_received_time_).seconds() * 1000.0;
  if(drive_sta_fb_report_delta_time_ms>param_.report_timeout_ms || drive_sta_fb_ptr_==nullptr)
  {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "drive stat fb report timeout = %f ms.", drive_sta_fb_report_delta_time_ms);
  }else{
    drive_sta_fb_pub_->publish(*drive_sta_fb_ptr_);
  }
  **/
  
    const double general_vehicle_status_report_delta_time_ms =
    (current_time - general_vehicle_status_received_time_).seconds() * 1000.0;
    if(general_vehicle_status_report_delta_time_ms>param_.report_timeout_ms || general_vehicle_status_ptr_==nullptr)
    {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "general_vehicle_status report timeout = %f ms.", general_vehicle_status_report_delta_time_ms);
    }else{
        general_vehicle_status_pub_->publish(*general_vehicle_status_ptr_);
    }
    
    const double steering_and_speed_report_delta_time_ms =
    (current_time - steering_and_speed_received_time_).seconds() * 1000.0;
    if(steering_and_speed_report_delta_time_ms>param_.report_timeout_ms || steering_and_speed_ptr_==nullptr)
    {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "steering_and_speed report timeout = %f ms.", steering_and_speed_report_delta_time_ms);
    }else{
        steering_and_speed_pub_->publish(*steering_and_speed_ptr_);
    }
    
    const double u_s_sensor_front_report_delta_time_ms =
    (current_time - u_s_sensor_front_received_time_).seconds() * 1000.0;
    if(u_s_sensor_front_report_delta_time_ms>param_.report_timeout_ms || u_s_sensor_front_ptr_==nullptr)
    {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "u_s_sensor_front report timeout = %f ms.", u_s_sensor_front_report_delta_time_ms);
    }else{
        u_s_sensor_front_pub_->publish(*u_s_sensor_front_ptr_);
    }
    
    const double u_s_sensor_rear_report_delta_time_ms =
    (current_time - u_s_sensor_rear_received_time_).seconds() * 1000.0;
    if(u_s_sensor_rear_report_delta_time_ms>param_.report_timeout_ms || u_s_sensor_rear_ptr_==nullptr)
    {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "u_s_sensor_rear report timeout = %f ms.", u_s_sensor_rear_report_delta_time_ms);
    }else{
        u_s_sensor_rear_pub_->publish(*u_s_sensor_rear_ptr_);
    }
    
    const double battery_status_report_delta_time_ms =
    (current_time - battery_status_received_time_).seconds() * 1000.0;
    if(battery_status_report_delta_time_ms>param_.report_timeout_ms || battery_status_ptr_==nullptr)
    {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "battery_status report timeout = %f ms.", battery_status_report_delta_time_ms);
    }else{
        battery_status_pub_->publish(*battery_status_ptr_);
    }
    
    const double raw_signal_throttle_report_delta_time_ms =
    (current_time - raw_signal_throttle_received_time_).seconds() * 1000.0;
    if(raw_signal_throttle_report_delta_time_ms>param_.report_timeout_ms || raw_signal_throttle_ptr_==nullptr)
    {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "raw_signal_throttle report timeout = %f ms.", raw_signal_throttle_report_delta_time_ms);
    }else{
        raw_signal_throttle_pub_->publish(*raw_signal_throttle_ptr_);
    }
    
    const double raw_signal_break_report_delta_time_ms =
    (current_time - raw_signal_break_received_time_).seconds() * 1000.0;
    if(raw_signal_break_report_delta_time_ms>param_.report_timeout_ms || raw_signal_break_ptr_==nullptr)
    {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "raw_signal_break report timeout = %f ms.", raw_signal_break_report_delta_time_ms);
    }else{
        raw_signal_break_pub_->publish(*raw_signal_break_ptr_);
    }
    
    const double raw_signal_steering_force_report_delta_time_ms =
    (current_time - raw_signal_steering_force_received_time_).seconds() * 1000.0;
    if(raw_signal_steering_force_report_delta_time_ms>param_.report_timeout_ms || raw_signal_steering_force_ptr_==nullptr)
    {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "raw_signal_steering_force report timeout = %f ms.", raw_signal_steering_force_report_delta_time_ms);
    }else{
        raw_signal_steering_force_pub_->publish(*raw_signal_steering_force_ptr_);
    }
    
    const double raw_signal_steering_position_report_delta_time_ms =
    (current_time - raw_signal_steering_position_received_time_).seconds() * 1000.0;
    if(raw_signal_steering_position_report_delta_time_ms>param_.report_timeout_ms || raw_signal_steering_position_ptr_==nullptr)
    {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "raw_signal_steering_position report timeout = %f ms.", raw_signal_steering_position_report_delta_time_ms);
    }else{
        raw_signal_steering_position_pub_->publish(*raw_signal_steering_position_ptr_);
    }
    
    const double scaled_signals_report_delta_time_ms =
    (current_time - scaled_signals_received_time_).seconds() * 1000.0;
    if(scaled_signals_report_delta_time_ms>param_.report_timeout_ms || scaled_signals_ptr_==nullptr)
    {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "scaled_signals report timeout = %f ms.", scaled_signals_report_delta_time_ms);
    }else{
        scaled_signals_pub_->publish(*scaled_signals_ptr_);
    }
    
    const double raw_signals_steering_and_speed_report_delta_time_ms =
    (current_time - raw_signals_steering_and_speed_received_time_).seconds() * 1000.0;
    if(raw_signals_steering_and_speed_report_delta_time_ms>param_.report_timeout_ms || raw_signals_steering_and_speed_ptr_==nullptr)
    {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "raw_signals_steering_and_speed report timeout = %f ms.", raw_signals_steering_and_speed_report_delta_time_ms);
    }else{
        raw_signals_steering_and_speed_pub_->publish(*raw_signals_steering_and_speed_ptr_);
    }
    
    const double temperatures_report_delta_time_ms =
    (current_time - temperatures_received_time_).seconds() * 1000.0;
    if(temperatures_report_delta_time_ms>param_.report_timeout_ms || temperatures_ptr_==nullptr)
    {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "temperatures report timeout = %f ms.", temperatures_report_delta_time_ms);
    }else{
        temperatures_pub_->publish(*temperatures_ptr_);
    }
    
    const double v_e_c_t_o_r___i_n_d_e_p_e_n_d_e_n_t__s_i_g__m_s_g_report_delta_time_ms =
    (current_time - v_e_c_t_o_r___i_n_d_e_p_e_n_d_e_n_t__s_i_g__m_s_g_received_time_).seconds() * 1000.0;
    if(v_e_c_t_o_r___i_n_d_e_p_e_n_d_e_n_t__s_i_g__m_s_g_report_delta_time_ms>param_.report_timeout_ms || v_e_c_t_o_r___i_n_d_e_p_e_n_d_e_n_t__s_i_g__m_s_g_ptr_==nullptr)
    {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "v_e_c_t_o_r___i_n_d_e_p_e_n_d_e_n_t__s_i_g__m_s_g report timeout = %f ms.", v_e_c_t_o_r___i_n_d_e_p_e_n_d_e_n_t__s_i_g__m_s_g_report_delta_time_ms);
    }else{
        v_e_c_t_o_r___i_n_d_e_p_e_n_d_e_n_t__s_i_g__m_s_g_pub_->publish(*v_e_c_t_o_r___i_n_d_e_p_e_n_d_e_n_t__s_i_g__m_s_g_ptr_);
    }
    
  
}

} // namespace report_parser
} // namespace pix_nina_driver
