#ifndef PIX_HOOKE_DRIVER__REPORT_PARSER_HPP_
#define PIX_HOOKE_DRIVER__REPORT_PARSER_HPP_

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>

#include <can_msgs/msg/frame.hpp>


// include- msgs header file
// Example: #include "pix_driver_msgs/BrakeCommand.h"
// #include pix_driver_msgs/protocols["name"].h
#include <pix_nina_driver_msgs/msg/general_vehicle_status.hpp>
#include <pix_nina_driver_msgs/msg/steering_and_speed.hpp>
#include <pix_nina_driver_msgs/msg/us_sensor_front.hpp>
#include <pix_nina_driver_msgs/msg/us_sensor_rear.hpp>
#include <pix_nina_driver_msgs/msg/battery_status.hpp>
#include <pix_nina_driver_msgs/msg/raw_signal_throttle.hpp>
#include <pix_nina_driver_msgs/msg/raw_signal_break.hpp>
#include <pix_nina_driver_msgs/msg/raw_signal_steering_force.hpp>
#include <pix_nina_driver_msgs/msg/raw_signal_steering_position.hpp>
#include <pix_nina_driver_msgs/msg/scaled_signals.hpp>
#include <pix_nina_driver_msgs/msg/raw_signals_steering_and_speed.hpp>
#include <pix_nina_driver_msgs/msg/temperatures.hpp>
#include <pix_nina_driver_msgs/msg/vectorindependentsigmsg.hpp>



// include- Parse header file
// Example: #include "brake_command_101.hpp"
// #include protocols["name"].cpp
#include <pix_nina_driver/general_vehicle_status.hpp>
#include <pix_nina_driver/steering_and_speed.hpp>
#include <pix_nina_driver/u_s_sensor_front.hpp>
#include <pix_nina_driver/u_s_sensor_rear.hpp>
#include <pix_nina_driver/battery_status.hpp>
#include <pix_nina_driver/raw_signal_throttle.hpp>
#include <pix_nina_driver/raw_signal_break.hpp>
#include <pix_nina_driver/raw_signal_steering_force.hpp>
#include <pix_nina_driver/raw_signal_steering_position.hpp>
#include <pix_nina_driver/scaled_signals.hpp>
#include <pix_nina_driver/raw_signals_steering_and_speed.hpp>
#include <pix_nina_driver/temperatures.hpp>
#include <pix_nina_driver/v_e_c_t_o_r___i_n_d_e_p_e_n_d_e_n_t__s_i_g__m_s_g.hpp>


namespace pix_nina_driver
{
namespace report_parser
{

/**
 * @brief param structure of report parser node
 * @param base_frame_id frame id of vehicle
 * @param loop_rate loop rate of publishers in hz
 * @param report_timeout_ms timeout threshold of report can Frame msg from canbus driver
 */
struct Param
{
  std::string base_frame_id;
  double loop_rate;
  int report_timeout_ms;
};

class ReportParser : public rclcpp::Node
{
private:
  // parameters of node
  Param param_;

  // is publish subscrber
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_publish_sub_;
  bool is_publish_;

  // subscribers from socketcan interface
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_frame_sub_;

  // publishers
  /** example
  rclcpp::Publisher<V2aBrakeStaFb>::SharedPtr brake_sta_fb_pub_;
  **/
  rclcpp::Publisher<pix_nina_driver_msgs::msg::GeneralVehicleStatus>::SharedPtr general_vehicle_status_pub_;
rclcpp::Publisher<pix_nina_driver_msgs::msg::SteeringAndSpeed>::SharedPtr steering_and_speed_pub_;
rclcpp::Publisher<pix_nina_driver_msgs::msg::USSensorFront>::SharedPtr u_s_sensor_front_pub_;
rclcpp::Publisher<pix_nina_driver_msgs::msg::USSensorRear>::SharedPtr u_s_sensor_rear_pub_;
rclcpp::Publisher<pix_nina_driver_msgs::msg::BatteryStatus>::SharedPtr battery_status_pub_;
rclcpp::Publisher<pix_nina_driver_msgs::msg::RawSignalThrottle>::SharedPtr raw_signal_throttle_pub_;
rclcpp::Publisher<pix_nina_driver_msgs::msg::RawSignalBreak>::SharedPtr raw_signal_break_pub_;
rclcpp::Publisher<pix_nina_driver_msgs::msg::RawSignalSteeringForce>::SharedPtr raw_signal_steering_force_pub_;
rclcpp::Publisher<pix_nina_driver_msgs::msg::RawSignalSteeringPosition>::SharedPtr raw_signal_steering_position_pub_;
rclcpp::Publisher<pix_nina_driver_msgs::msg::ScaledSignals>::SharedPtr scaled_signals_pub_;
rclcpp::Publisher<pix_nina_driver_msgs::msg::RawSignalsSteeringAndSpeed>::SharedPtr raw_signals_steering_and_speed_pub_;
rclcpp::Publisher<pix_nina_driver_msgs::msg::Temperatures>::SharedPtr temperatures_pub_;
rclcpp::Publisher<pix_nina_driver_msgs::msg::VECTORINDEPENDENTSIGMSG>::SharedPtr v_e_c_t_o_r___i_n_d_e_p_e_n_d_e_n_t__s_i_g__m_s_g_pub_;


  // publish msgs
  /** example
  V2aBrakeStaFb::ConstSharedPtr brake_sta_fb_ptr_;
  **/
  pix_nina_driver_msgs::msg::GeneralVehicleStatus::ConstSharedPtr general_vehicle_status_ptr_;
pix_nina_driver_msgs::msg::SteeringAndSpeed::ConstSharedPtr steering_and_speed_ptr_;
pix_nina_driver_msgs::msg::USSensorFront::ConstSharedPtr u_s_sensor_front_ptr_;
pix_nina_driver_msgs::msg::USSensorRear::ConstSharedPtr u_s_sensor_rear_ptr_;
pix_nina_driver_msgs::msg::BatteryStatus::ConstSharedPtr battery_status_ptr_;
pix_nina_driver_msgs::msg::RawSignalThrottle::ConstSharedPtr raw_signal_throttle_ptr_;
pix_nina_driver_msgs::msg::RawSignalBreak::ConstSharedPtr raw_signal_break_ptr_;
pix_nina_driver_msgs::msg::RawSignalSteeringForce::ConstSharedPtr raw_signal_steering_force_ptr_;
pix_nina_driver_msgs::msg::RawSignalSteeringPosition::ConstSharedPtr raw_signal_steering_position_ptr_;
pix_nina_driver_msgs::msg::ScaledSignals::ConstSharedPtr scaled_signals_ptr_;
pix_nina_driver_msgs::msg::RawSignalsSteeringAndSpeed::ConstSharedPtr raw_signals_steering_and_speed_ptr_;
pix_nina_driver_msgs::msg::Temperatures::ConstSharedPtr temperatures_ptr_;
pix_nina_driver_msgs::msg::VECTORINDEPENDENTSIGMSG::ConstSharedPtr v_e_c_t_o_r___i_n_d_e_p_e_n_d_e_n_t__s_i_g__m_s_g_ptr_;


  // can frame entities
  /** example
  V2adrivestafb530  v2a_drivestafb_530_entity_;
  **/
  GeneralVehicleStatus general_vehicle_status_entity_;
SteeringAndSpeed steering_and_speed_entity_;
USSensorFront u_s_sensor_front_entity_;
USSensorRear u_s_sensor_rear_entity_;
BatteryStatus battery_status_entity_;
RawSignalThrottle raw_signal_throttle_entity_;
RawSignalBreak raw_signal_break_entity_;
RawSignalSteeringForce raw_signal_steering_force_entity_;
RawSignalSteeringPosition raw_signal_steering_position_entity_;
ScaledSignals scaled_signals_entity_;
RawSignalsSteeringAndSpeed raw_signals_steering_and_speed_entity_;
Temperatures temperatures_entity_;
VECTORINDEPENDENTSIGMSG v_e_c_t_o_r___i_n_d_e_p_e_n_d_e_n_t__s_i_g__m_s_g_entity_;


  // msg reveived time
  /** example
  rclcpp::Time brake_sta_fb_received_time_;
  **/
  rclcpp::Time general_vehicle_status_received_time_;
rclcpp::Time steering_and_speed_received_time_;
rclcpp::Time u_s_sensor_front_received_time_;
rclcpp::Time u_s_sensor_rear_received_time_;
rclcpp::Time battery_status_received_time_;
rclcpp::Time raw_signal_throttle_received_time_;
rclcpp::Time raw_signal_break_received_time_;
rclcpp::Time raw_signal_steering_force_received_time_;
rclcpp::Time raw_signal_steering_position_received_time_;
rclcpp::Time scaled_signals_received_time_;
rclcpp::Time raw_signals_steering_and_speed_received_time_;
rclcpp::Time temperatures_received_time_;
rclcpp::Time v_e_c_t_o_r___i_n_d_e_p_e_n_d_e_n_t__s_i_g__m_s_g_received_time_;


  // timer
  rclcpp::TimerBase::SharedPtr timer_;

public:
  ReportParser();
  // callback
  /**
   * @brief callback function of can Frame msgs, to store the data to member variable
   * 
   * @param msg 
   */
  void callbackCan(const can_msgs::msg::Frame::ConstSharedPtr & msg);
  /**
   * @brief callback function of Bool msg, to store the data to member variable, decide publish report msgs or not
   * 
   * @param msg 
   */
  void callbackIsPublish(const std_msgs::msg::Bool::ConstSharedPtr & msg);
  /**
   * @brief parser can frames, convert can frames to pix_driver_msgs
   * 
   */
  void timerCallback();
};
} // report_parser
} // pix_nina_driver
#endif // PIX_HOOKE_DRIVER__REPORT_PARSER_HPP_