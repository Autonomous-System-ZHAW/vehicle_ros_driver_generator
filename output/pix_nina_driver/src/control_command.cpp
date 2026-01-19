#include <pix_nina_driver/control_command.hpp>

namespace pix_nina_driver
{
namespace control_command
{
ControlCommand::ControlCommand() : Node("control_command")
{
  // ros params
  param_.base_frame_id = declare_parameter("base_frame_id", "base_link");
  param_.command_timeout_ms = declare_parameter("command_timeout_ms", 1000);
  param_.loop_rate = declare_parameter("loop_rate", 50.0);

  // initialize msg received time, make reservation of data size
  // example brake_command_received_time_ = this->now();
  remote_application_toggle_request_received_time_ = this->now();remote_indicator_request_received_time_ = this->now();remote_drive_request_received_time_ = this->now();

  is_engage_ = true;

  using std::placeholders::_1;

  /* subscriber */
  {
    // from nina driver autoware interface
    /**
    a2v_brake_ctrl_sub_ = create_subscription<A2vBrakeCtrl>(
      "/pix_hooke/a2v_brakectrl_131", 1, std::bind(&ControlCommand::callbackBrakeCtrl, this, _1));
    **/
    remote_application_toggle_request_sub_ = create_subscription<pix_nina_driver_msgs::msg::RemoteApplicationToggleRequest>("/pix_nina/remote_application_toggle_request", 1, std::bind(&ControlCommand::callbackRemoteApplicationToggleRequest, this, _1));
remote_indicator_request_sub_ = create_subscription<pix_nina_driver_msgs::msg::RemoteIndicatorRequest>("/pix_nina/remote_indicator_request", 1, std::bind(&ControlCommand::callbackRemoteIndicatorRequest, this, _1));
remote_drive_request_sub_ = create_subscription<pix_nina_driver_msgs::msg::RemoteDriveRequest>("/pix_nina/remote_drive_request", 1, std::bind(&ControlCommand::callbackRemoteDriveRequest, this, _1));

    // engage
    engage_ctrl_sub_ = create_subscription<std_msgs::msg::Bool>(
      "input/engage", 1, std::bind(&ControlCommand::callbackEngage, this, _1));
  }
  /* publisher */
  {
    // to socketcan drivier
    can_frame_pub_ = create_publisher<can_msgs::msg::Frame>("output/can_tx", rclcpp::QoS{1});
  }
  {
    // timer
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Rate(param_.loop_rate).period(),
      std::bind(&ControlCommand::timerCallback, this));
  }
}

// calback functions
/** example
void ControlCommand::callbackBrakeCtrl(const A2vBrakeCtrl::ConstSharedPtr & msg)
{
  brake_command_received_time_ = this->now();
  brake_ctrl_ptr_ = msg;
  a2v_brakectrl_131_entity_.Reset();
  a2v_brakectrl_131_entity_.UpdateData(
    msg->acu_chassis_brake_en, msg->acu_chassis_aeb_ctrl, msg->acu_chassis_brake_pdl_target,
    msg->acu_chassis_epb_ctrl, msg->acu_brake_life_sig, msg->acu_check_sum_131);
  can_msgs::msg::Frame brake_ctrl_can_msg;
  brake_ctrl_can_msg.header.stamp = msg->header.stamp;
  brake_ctrl_can_msg.dlc = 8;
  brake_ctrl_can_msg.id = a2v_brakectrl_131_entity_.ID;
  brake_ctrl_can_msg.is_extended = false;
  uint8_t *signal_bits;
  signal_bits = a2v_brakectrl_131_entity_.get_data();
  for (int i = 0; i < 8; i++)
  {
    brake_ctrl_can_msg.data[i] = *signal_bits;
    signal_bits += 1;
  }
  brake_ctrl_can_ptr_ = std::make_shared<can_msgs::msg::Frame>(brake_ctrl_can_msg);
}
**/


    void ControlCommand::callbackRemoteApplicationToggleRequest(const pix_nina_driver_msgs::msg::RemoteApplicationToggleRequest::ConstSharedPtr & msg)
    {
        remote_application_toggle_request_received_time_ = this->now();
        remote_application_toggle_request_ptr_ = msg;
        remote_application_toggle_request_entity_.Reset();
        remote_application_toggle_request_entity_.UpdateData(msg->app_toggle_req__speed_limit4kmh, msg->app_toggle_req__send_c_a_n_dbg_messages, msg->app_toggle_req__pwr_assisted_braking);
        can_msgs::msg::Frame remote_application_toggle_request_can_msg;
        remote_application_toggle_request_can_msg.header.stamp = msg->header.stamp;
        remote_application_toggle_request_can_msg.dlc = 8;
        remote_application_toggle_request_can_msg.id = remote_application_toggle_request_entity_.ID;
        remote_application_toggle_request_can_msg.is_extended = false;
        uint8_t *signal_bits;
        signal_bits = remote_application_toggle_request_entity_.get_data();
        for (int i = 0; i < 8; i++)
        {
            remote_application_toggle_request_can_msg.data[i] = *signal_bits;
            signal_bits += 1;
        }
        remote_application_toggle_request_can_ptr_ = std::make_shared<can_msgs::msg::Frame>(remote_application_toggle_request_can_msg);
    }

    

    void ControlCommand::callbackRemoteIndicatorRequest(const pix_nina_driver_msgs::msg::RemoteIndicatorRequest::ConstSharedPtr & msg)
    {
        remote_indicator_request_received_time_ = this->now();
        remote_indicator_request_ptr_ = msg;
        remote_indicator_request_entity_.Reset();
        remote_indicator_request_entity_.UpdateData(msg->remote_status_light_blink_rate_req, msg->remote_status_light_color_req, msg->remote_horn_req, msg->remote_light_brake_req, msg->remote_blink_left_req, msg->remote_blink_right_req, msg->remote_brights_on_req, msg->remote_light_reverse_req, msg->remote_lights_turn_left_req, msg->remote_lights_turn_right_req);
        can_msgs::msg::Frame remote_indicator_request_can_msg;
        remote_indicator_request_can_msg.header.stamp = msg->header.stamp;
        remote_indicator_request_can_msg.dlc = 8;
        remote_indicator_request_can_msg.id = remote_indicator_request_entity_.ID;
        remote_indicator_request_can_msg.is_extended = false;
        uint8_t *signal_bits;
        signal_bits = remote_indicator_request_entity_.get_data();
        for (int i = 0; i < 8; i++)
        {
            remote_indicator_request_can_msg.data[i] = *signal_bits;
            signal_bits += 1;
        }
        remote_indicator_request_can_ptr_ = std::make_shared<can_msgs::msg::Frame>(remote_indicator_request_can_msg);
    }

    

    void ControlCommand::callbackRemoteDriveRequest(const pix_nina_driver_msgs::msg::RemoteDriveRequest::ConstSharedPtr & msg)
    {
        remote_drive_request_received_time_ = this->now();
        remote_drive_request_ptr_ = msg;
        remote_drive_request_entity_.Reset();
        remote_drive_request_entity_.UpdateData(msg->remote_steering_angle_req, msg->remote_velocity_req);
        can_msgs::msg::Frame remote_drive_request_can_msg;
        remote_drive_request_can_msg.header.stamp = msg->header.stamp;
        remote_drive_request_can_msg.dlc = 8;
        remote_drive_request_can_msg.id = remote_drive_request_entity_.ID;
        remote_drive_request_can_msg.is_extended = false;
        uint8_t *signal_bits;
        signal_bits = remote_drive_request_entity_.get_data();
        for (int i = 0; i < 8; i++)
        {
            remote_drive_request_can_msg.data[i] = *signal_bits;
            signal_bits += 1;
        }
        remote_drive_request_can_ptr_ = std::make_shared<can_msgs::msg::Frame>(remote_drive_request_can_msg);
    }

    

void ControlCommand::callbackEngage(const std_msgs::msg::Bool::ConstSharedPtr & msg)
{
  is_engage_ = msg->data;
}

void ControlCommand::timerCallback()
{
  if (!is_engage_) return;
  const rclcpp::Time current_time = this->now();

  // publishing msg
  /** example
  // brake control command 
  const double brake_command_delta_time_ms =
    (current_time - brake_command_received_time_).seconds() * 1000.0;
  if (brake_command_delta_time_ms > param_.command_timeout_ms || brake_ctrl_can_ptr_==nullptr) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "brake command timeout = %f ms.", brake_command_delta_time_ms);
  } else {
    can_frame_pub_->publish(*brake_ctrl_can_ptr_);
  }
  **/
  

    // remote_application_toggle_request
    const double remote_application_toggle_request_delta_time_ms =
        (current_time - remote_application_toggle_request_received_time_).seconds() * 1000.0;
    if (remote_application_toggle_request_delta_time_ms > param_.command_timeout_ms || remote_application_toggle_request_can_ptr_==nullptr) {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "remote_application_toggle_request timeout = %f ms.", remote_application_toggle_request_delta_time_ms);
    } else {
        can_frame_pub_->publish(*remote_application_toggle_request_can_ptr_);
    }

    

    // remote_indicator_request
    const double remote_indicator_request_delta_time_ms =
        (current_time - remote_indicator_request_received_time_).seconds() * 1000.0;
    if (remote_indicator_request_delta_time_ms > param_.command_timeout_ms || remote_indicator_request_can_ptr_==nullptr) {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "remote_indicator_request timeout = %f ms.", remote_indicator_request_delta_time_ms);
    } else {
        can_frame_pub_->publish(*remote_indicator_request_can_ptr_);
    }

    

    // remote_drive_request
    const double remote_drive_request_delta_time_ms =
        (current_time - remote_drive_request_received_time_).seconds() * 1000.0;
    if (remote_drive_request_delta_time_ms > param_.command_timeout_ms || remote_drive_request_can_ptr_==nullptr) {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "remote_drive_request timeout = %f ms.", remote_drive_request_delta_time_ms);
    } else {
        can_frame_pub_->publish(*remote_drive_request_can_ptr_);
    }

    
}

} // namespace control_command
} // namespace pix_nina_driver
