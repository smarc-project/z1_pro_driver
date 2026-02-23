/**
 * @author Aldo Teran Espinoza
 * @author_email aldot@kth.se
 */
#include <chrono>
#include <iostream>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geographic_msgs/msg/geo_point.hpp"
#include "z1_pro_msgs/msg/cam_cmd.hpp"

#include "GeographicLib/UTMUPS.hpp"

using namespace std::chrono_literals;

class GimbalInterface : public rclcpp::Node {
 public:
  GimbalInterface() : Node("gimbal_interface_node") {

    // Get topic names.
    this->declare_parameter<std::string>("cmd_topic");
    this->declare_parameter<std::string>("gimbal_ctrl_topic");
    this->declare_parameter<std::string>("odom_topic");
    this->declare_parameter<std::string>("geopoint_topic");
    this->declare_parameter<bool>("use_vehicle_altitude", false);
    const std::string cam_cmd_topic =
        this->get_parameter("cmd_topic").as_string();
    const std::string ctrl_topic =
        this->get_parameter("gimbal_ctrl_topic").as_string();
    const std::string odom_topic =
        this->get_parameter("odom_topic").as_string();
    const std::string geopoint_topic =
        this->get_parameter("geopoint_topic").as_string();
    use_vehicle_altitude_ =
        this->get_parameter("use_vehicle_altitude").as_bool();

    cam_angles_pub_ =
        this->create_publisher<geometry_msgs::msg::Vector3>(ctrl_topic, 10);

    cam_cmd_sub_ = this->create_subscription<z1_pro_msgs::msg::CamCmd>(
        cam_cmd_topic, 10,
        std::bind(&GimbalInterface::CamCmdCallback, this,
                  std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10,
        std::bind(&GimbalInterface::OdometryCallback, this,
                  std::placeholders::_1));

    geopoint_sub_ = this->create_subscription<geographic_msgs::msg::GeoPoint>(
        geopoint_topic, 10,
        std::bind(&GimbalInterface::GeoPointCallback, this,
                  std::placeholders::_1));

    // FIXME: publishing rate fixed to 50Hz.
    // Timer for checking for and publishing updates..
    publish_timer_ = this->create_wall_timer(
        20ms, std::bind(&GimbalInterface::PublishTimerCallback, this));

  }

 private:
  // Constants for indexing.
  int ROLL = 0;
  int PITCH = 1;
  int YAW = 2;
  int FRAME = 3;
  int POI = 4;

  // Limits for the camera angles.
  double MAX_ROLL = 50.0;   // [deg]
  double MAX_PITCH = 100.0; // [deg]
  double MAX_YAW = 150.0;   // [deg]

  // Track the state of Evolo.
  Eigen::Vector3d w_trans_e_w_ = Eigen::Vector3d(0, 0, 0); // [m] UTM position.
  Eigen::Quaterniond w_rot_e_ = Eigen::Quaterniond(1, 0, 0, 0);
  bool use_vehicle_altitude_ = false;

  // Track the desired control output for the camera.
  double desired_roll_ = 0.0;   // [rad]
  double desired_pitch_ = 0.0;  // [rad]
  double desired_yaw_ = 0.0;    // [rad]

  // Track POI.
  double x_poi_; // [m] UTM X position of the POI.
  double y_poi_; // [m] UTM y position of the POI.
  double z_poi_; // [m] UTM z position of the POI.
  bool tracking_poi_ = false;

  // Keep track of the last CamCmd message sent.
  z1_pro_msgs::msg::CamCmd::SharedPtr prev_cmd_msg_;
  bool camera_init_ = false;

  Eigen::Matrix3d R_NED_to_ENU =
      (Eigen::Matrix3d() << 0, 1, 0, 1, 0, 0, 0, 0, -1).finished();

  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr cam_angles_pub_;
  rclcpp::Subscription<z1_pro_msgs::msg::CamCmd>::SharedPtr cam_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geographic_msgs::msg::GeoPoint>::SharedPtr geopoint_sub_;

  rclcpp::TimerBase::SharedPtr publish_timer_;


  // -----------------------------------------------------------------------
  // Check which message fields have changed between callbacks.
  void _check_delta_msg(const z1_pro_msgs::msg::CamCmd::SharedPtr msg,
                        std::vector<int>& changes) {
    // Check roll, pitch, yaw.
    changes.push_back((prev_cmd_msg_->roll == msg->roll ) ? 0 : 1);
    changes.push_back((prev_cmd_msg_->pitch == msg->pitch ) ? 0 : 1);
    changes.push_back((prev_cmd_msg_->yaw == msg->yaw ) ? 0 : 1);

    // Check frame change.
    changes.push_back((prev_cmd_msg_->frame == msg->frame) ? 0 : 1);

    // Check POI change.
    int lat_change =
        (prev_cmd_msg_->poi.latitude == msg->poi.latitude ) ? 0 : 1;
    int lon_change =
        (prev_cmd_msg_->poi.longitude == msg->poi.longitude ) ? 0 : 1;
    int alt_change =
        (prev_cmd_msg_->poi.altitude == msg->poi.altitude ) ? 0 : 1;
    changes.push_back(lat_change || lon_change || alt_change);

    // TODO: channel and resolution?
    
    prev_cmd_msg_ = msg;
  }


  // -----------------------------------------------------------------------
  // Update for the point of interest (POI) callback.
  void _update_poi(const z1_pro_msgs::msg::CamCmd::SharedPtr msg){
    // Get target's UTM.
    int zone;
    bool northp;
    GeographicLib::UTMUPS::Forward(msg->poi.latitude, msg->poi.longitude, zone,
                                   northp, x_poi_, y_poi_);
    z_poi_ = msg->poi.altitude;

    tracking_poi_ = true;
  }


  // -----------------------------------------------------------------------
  // Update desired angles wrt our current position and the POI.
  void _track_poi(){

    const Eigen::Vector3d w_trans_poi_w(x_poi_, y_poi_, z_poi_);

    // Relative position vector.
    const Eigen::Matrix3d e_rot_w = w_rot_e_.toRotationMatrix().transpose();
    Eigen::Vector3d e_trans_poi_e =
        -e_rot_w * w_trans_e_w_ + e_rot_w * w_trans_poi_w;
    const double yaw =
        std::atan2(e_trans_poi_e(1), e_trans_poi_e(0)) * 180.0 / M_PI;
    const double pitch =
        -std::atan2(e_trans_poi_e(2),
                    std::sqrt(e_trans_poi_e(0) * e_trans_poi_e(0) +
                              e_trans_poi_e(1) * e_trans_poi_e(1))) *
        180.0 / M_PI;
    // Limit.
    desired_yaw_ =
        (std::abs(yaw) <= MAX_YAW) ? yaw : MAX_YAW * (yaw / std::abs(yaw));
    desired_pitch_ = (std::abs(pitch) <= MAX_PITCH)
                        ? pitch
                        : MAX_PITCH * (pitch / std::abs(pitch));

    tracking_poi_ = true;
  }


  // -----------------------------------------------------------------------
  // Update desired roll angle.
  void _update_roll(const z1_pro_msgs::msg::CamCmd::SharedPtr msg) {
    double roll;
    if (msg->frame == msg->GLOBAL) {
      Eigen::Matrix3d w_rot_c;
      w_rot_c = Eigen::AngleAxisd(msg->roll * M_PI / 180.0,
                                  Eigen::Vector3d::UnitX());
      const Eigen::Matrix3d e_rot_c =
          w_rot_c.transpose() * w_rot_e_.toRotationMatrix();
      const Eigen::Vector3d euler = e_rot_c.eulerAngles(0, 1, 2);
      roll = euler(0) * 180.0 / M_PI;
    } else {
      roll = msg->roll;
    }

    // Limit.
    desired_roll_ = (std::abs(roll) <= MAX_ROLL)
                       ? roll
                       : MAX_ROLL * (roll / std::abs(roll));
  }

  // -----------------------------------------------------------------------
  // Update desired pitch angles.
  void _update_pitch(const z1_pro_msgs::msg::CamCmd::SharedPtr msg) {
    double pitch;
    if (msg->frame == msg->GLOBAL) {
      Eigen::Matrix3d w_rot_c;
      w_rot_c = Eigen::AngleAxisd(msg->pitch * M_PI / 180.0,
                                  Eigen::Vector3d::UnitY());
      const Eigen::Matrix3d e_rot_c =
          w_rot_c.transpose() * w_rot_e_.toRotationMatrix();
      const Eigen::Vector3d euler = e_rot_c.eulerAngles(0, 1, 2);
      pitch = euler(1) * 180.0 / M_PI;
    } else {
      pitch = msg->pitch;
    }
    // Limit.
    desired_pitch_ = (std::abs(pitch) <= MAX_PITCH)
                        ? pitch
                        : MAX_PITCH * (pitch / std::abs(pitch));
  }

  // -----------------------------------------------------------------------
  // Update desired yaw angle.
  void _update_yaw(const z1_pro_msgs::msg::CamCmd::SharedPtr msg) {
    double yaw;
    if (msg->frame == msg->GLOBAL) {
      // Global yaw comes as compass heading (NED) from 0-360 deg.
      const double w_yaw_c = std::atan2(std::cos(msg->yaw * M_PI / 180.0),
                                        std::sin(msg->yaw * M_PI / 180.0));
      Eigen::Matrix3d w_rot_c;
      w_rot_c = Eigen::AngleAxisd(w_yaw_c, Eigen::Vector3d::UnitZ());
      const Eigen::Matrix3d e_rot_c =
         w_rot_e_.toRotationMatrix().transpose() * w_rot_c;
      const Eigen::Vector3d euler = e_rot_c.eulerAngles(0, 1, 2);
      yaw = euler(2) * 180.0 / M_PI;
    } else {
      yaw = msg->yaw;
    }
    // Limit.
    desired_yaw_ = (std::abs(yaw) <= MAX_YAW)
                      ? yaw 
                      : MAX_YAW * (yaw / std::abs(yaw));

    // Yaw cmd overrides POI.
    tracking_poi_ = false;
  }

  // -----------------------------------------------------------------------
  // Update the desired control message.
  void CamCmdCallback(const z1_pro_msgs::msg::CamCmd::SharedPtr msg) {
    if (!camera_init_){
      prev_cmd_msg_ = msg;
      camera_init_ = true;
    }

    // Check for changes.
    std::vector<int> changes;
    _check_delta_msg(msg, changes);
    
    // Check changes in a heirarchical way.
    // TODO: Is it too ineficient to update angles individually?
    if (changes[POI]){
      // POI takes priority over anything else.
      _update_poi(msg);
    } else if (changes[YAW]){
      // If something wants to take over yaw, we'll let them as long as there's
      // no new POI.
      _update_yaw(msg);
      //_update_pitch(msg);
    }

    if (!tracking_poi_){
      _update_pitch(msg);
    }

    // Always track user's roll.
    _update_roll(msg);

    // TODO: Always update channel and resolution.
    //channel = msg.channel;
    //resolution = msg.resolution;
  }


  // -----------------------------------------------------------------------
  // Here we'll only update the position of Evolo in UTM.
  void GeoPointCallback(const geographic_msgs::msg::GeoPoint::SharedPtr msg) {
    // Get lat/lon in UTM
    double x_pos, y_pos;
    int zone;
    bool northp;
    GeographicLib::UTMUPS::Forward(msg->latitude, msg->longitude, zone, northp,
                                   x_pos, y_pos);

    // 2D position if no altitude is required.
    if (use_vehicle_altitude_){
      w_trans_e_w_ = Eigen::Vector3d(x_pos, y_pos, msg->altitude);
    }else {
      w_trans_e_w_ = Eigen::Vector3d(x_pos, y_pos, 0.0);
    }
  }


  // -----------------------------------------------------------------------
  // Here we'll only update the orientation of Evolo in ENU.
  void OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;

    // Convert quaternion to rotation matrix
    w_rot_e_ = Eigen::Quaterniond(qw, qx, qy, qz);
  }

  geometry_msgs::msg::Vector3 _angles_to_message() {
    geometry_msgs::msg::Vector3 msg;
    msg.x = desired_roll_;
    msg.y = desired_pitch_;
    msg.z = desired_yaw_;
    // TODO: channel and resolution.
    
    return msg;
  }

  // -----------------------------------------------------------------------
  void PublishTimerCallback() {
    if (tracking_poi_){
      _track_poi();
    }
    cam_angles_pub_->publish(_angles_to_message());
  }
};

// -----------------------------------------------------------------------
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  std::cout << "Starting the gimbal high-level interface node.\n";
  rclcpp::spin(std::make_shared<GimbalInterface>());

  rclcpp::shutdown();

  return 0;
}
