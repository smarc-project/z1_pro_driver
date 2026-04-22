/**
 * @author Aldo Teran Espinoza
 * @author_email aldot@kth.se
 */
#include <iostream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "z1_pro_msgs/msg/gcudata.hpp"

using namespace std::chrono_literals;

class GimbalJointPublisher : public rclcpp::Node {
 public:
  GimbalJointPublisher() : Node("gimbal_joint_publisher") {

    // TODO: get link names and topic names.
    this->declare_parameter<std::string>("gimbal_feedback_topic");
    const std::string feedback_topic =
        this->get_parameter("gimbal_feedback_topic").as_string();

    this->declare_parameter<bool>("camera_below_base", false);
    camera_below_base = this->get_parameter("camera_below_base").as_bool();

    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states", 10);

    gcudata_sub_ = this->create_subscription<z1_pro_msgs::msg::Gcudata>(
        feedback_topic, 10,
        std::bind(&GimbalJointPublisher::GcudataCallback, this,
                  std::placeholders::_1));

    joint_msg_.name = {"yaw_joint", "roll_joint", "pitch_joint"};
    joint_msg_.position = {0.0, 0.0, 0.0};

    // Setup a timer to publish the states faster than the camera's feedback.
    publish_timer_ = this->create_wall_timer(
        20ms /*50Hz*/, std::bind(&GimbalJointPublisher::TimerCallback, this));
  }


 private:
  // To keep track of gimbal feedback.
  double roll_angle_ = 0.0;  // [rad]
  double pitch_angle_ = 0.0; // [rad]
  double yaw_angle_ = 0.0;   // [rad]
  bool camera_below_base = false;

  rclcpp::Subscription<z1_pro_msgs::msg::Gcudata>::SharedPtr gcudata_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  sensor_msgs::msg::JointState joint_msg_;

  // -----------------------------------------------------------------------
  void GcudataCallback(const z1_pro_msgs::msg::Gcudata::SharedPtr msg) {
    // Just store the angles for the timer to publish.
    roll_angle_ = msg->relative_roll * M_PI / 180.0;
    pitch_angle_ = msg->relative_pitch * M_PI / 180.0;
    yaw_angle_ = msg->relative_yaw * M_PI / 180.0;
  }


  // -----------------------------------------------------------------------
  void TimerCallback() {
    // Build message and publish.
    joint_msg_.header.stamp = this->get_clock()->now();
    if(camera_below_base) {
      joint_msg_.position[0] = -yaw_angle_;
      joint_msg_.position[1] = roll_angle_;
      joint_msg_.position[2] = -pitch_angle_;
    } else {
      joint_msg_.position[0] = yaw_angle_;
      joint_msg_.position[1] = roll_angle_;
      joint_msg_.position[2] = pitch_angle_;
    }

    joint_pub_->publish(joint_msg_);
  }

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GimbalJointPublisher>());
  rclcpp::shutdown();

  return 0;
}
