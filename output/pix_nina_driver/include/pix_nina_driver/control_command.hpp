#ifndef PIX_HOOKE_DRIVER__CONTROL_COMMAND_HPP_
#define PIX_HOOKE_DRIVER__CONTROL_COMMAND_HPP_

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>

#include <can_msgs/msg/frame.hpp>


// include- msgs header file
// Example: #include "pix_driver_msgs/BrakeCommand.h"
// #include pix_driver_msgs/protocols["name"].h
#include <pix_nina_driver_msgs/msg/remote_application_toggle_request.hpp>
#include <pix_nina_driver_msgs/msg/remote_indicator_request.hpp>
#include <pix_nina_driver_msgs/msg/remote_drive_request.hpp>



// include- Parse header file
// Example: #include "brake_command_101.hpp"
// #include protocols["name"].cpp
#include <pix_nina_driver/remote_application_toggle_request.hpp>
#include <pix_nina_driver/remote_indicator_request.hpp>
#include <pix_nina_driver/remote_drive_request.hpp>


namespace pix_nina_driver
{
namespace control_command
{

/**
 * @brief param structure of control command node
 * @param base_frame_id frame id of vehicle
 * @param loop_rate loop rate of publishers in hz
 * @param command_timeout_ms timeout threshold of control command msg from control converter in ms
 */
struct Param
{
  std::string base_frame_id;
  double loop_rate;
  int command_timeout_ms;
};

class ControlCommand : public rclcpp::Node
{
private:
  // parameters of node
  Param param_;

  // subscribers
  // example rclcpp::Subscription<A2vBrakeCtrl>::SharedPtr a2v_brake_ctrl_sub_;
  rclcpp::Subscription<pix_nina_driver_msgs::msg::RemoteApplicationToggleRequest>::SharedPtr remote_application_toggle_request_sub_;
rclcpp::Subscription<pix_nina_driver_msgs::msg::RemoteIndicatorRequest>::SharedPtr remote_indicator_request_sub_;
rclcpp::Subscription<pix_nina_driver_msgs::msg::RemoteDriveRequest>::SharedPtr remote_drive_request_sub_;


  // msgs
  // example A2vBrakeCtrl::ConstSharedPtr brake_ctrl_ptr_;
  pix_nina_driver_msgs::msg::RemoteApplicationToggleRequest::ConstSharedPtr remote_application_toggle_request_ptr_;
pix_nina_driver_msgs::msg::RemoteIndicatorRequest::ConstSharedPtr remote_indicator_request_ptr_;
pix_nina_driver_msgs::msg::RemoteDriveRequest::ConstSharedPtr remote_drive_request_ptr_;


  // control command structures
  // example A2vdrivectrl130 a2v_drivectrl_130_entity_;
  RemoteApplicationToggleRequest remote_application_toggle_request_entity_;
RemoteIndicatorRequest remote_indicator_request_entity_;
RemoteDriveRequest remote_drive_request_entity_;


  // msg received timestamp
  // example rclcpp::Time drive_command_received_time_;
  rclcpp::Time remote_application_toggle_request_received_time_;
rclcpp::Time remote_indicator_request_received_time_;
rclcpp::Time remote_drive_request_received_time_;


  // state control
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr engage_ctrl_sub_;
  bool is_engage_;

  // publishers to can card driver
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_frame_pub_;

  // publishing can msgs
  // example can_msgs::msg::Frame::ConstSharedPtr brake_ctrl_can_ptr_;
  can_msgs::msg::Frame::ConstSharedPtr remote_application_toggle_request_can_ptr_;
can_msgs::msg::Frame::ConstSharedPtr remote_indicator_request_can_ptr_;
can_msgs::msg::Frame::ConstSharedPtr remote_drive_request_can_ptr_;


  // timer
  rclcpp::TimerBase::SharedPtr timer_;

public:
  ControlCommand();
  // calback functions
  // example
  // void callbackBrakeCtrl(const A2vBrakeCtrl::ConstSharedPtr & msg);
  void callbackRemoteApplicationToggleRequest(const pix_nina_driver_msgs::msg::RemoteApplicationToggleRequest::ConstSharedPtr & msg);

void callbackRemoteIndicatorRequest(const pix_nina_driver_msgs::msg::RemoteIndicatorRequest::ConstSharedPtr & msg);

void callbackRemoteDriveRequest(const pix_nina_driver_msgs::msg::RemoteDriveRequest::ConstSharedPtr & msg);


  void callbackEngage(const std_msgs::msg::Bool::ConstSharedPtr & msg);
  void timerCallback();

};

} // control_command
} // pix_nina_driver
#endif // PIX_HOOKE_DRIVER__CONTROL_COMMAND_HPP_