#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <mc_control/mc_global_controller.h>
#include <mc_rbdyn/Robot.h>
#include <mc_tasks/PostureTask.h>
#include <mc_tasks/CoMTask.h>

#include <Eigen/Geometry>
#include <map>
#include <mutex>
#include <string>
#include <vector>

class McRtcBridge : public rclcpp::Node
{
public:
  McRtcBridge();

  // Public API for programmatic use
  void setPostureTarget(const std::map<std::string, std::vector<double>> & targets);
  void setComOffset(const Eigen::Vector3d & offset);
  bool isInitialized() const;
  const std::vector<std::string> & refJointOrder() const;

private:
  // mc_rtc
  std::unique_ptr<mc_control::MCGlobalController> gc_;
  std::vector<std::string> ref_joint_order_;

  // ROS 2
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr posture_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr com_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  std::mutex state_mutex_;
  bool js_received_ = false;
  bool initialized_ = false;
  bool imu_received_ = false;

  // Joint name -> index in JointState message
  std::map<std::string, size_t> js_name_to_idx_;

  // Encoder values in mc_rtc ref_joint_order
  std::vector<double> encoder_positions_;
  std::vector<double> encoder_velocities_;

  // IMU data
  Eigen::Quaterniond imu_orientation_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d imu_angular_velocity_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d imu_linear_acceleration_ = Eigen::Vector3d::Zero();

  // Output mapping: whole_body_controller joint order
  std::vector<std::string> wbc_joint_order_;
  std::vector<int> mcrtc_to_wbc_;
  std::map<size_t, double> passthrough_positions_;

  // Passthrough joint names
  std::vector<std::string> passthrough_joints_;

  // Pending posture targets from topic
  std::map<std::string, std::vector<double>> pending_posture_targets_;

  // Pending CoM target (absolute position)
  bool com_target_pending_ = false;
  Eigen::Vector3d pending_com_target_ = Eigen::Vector3d::Zero();

  // Startup ramp interpolation
  bool ramping_ = false;
  double ramp_duration_ = 3.0;  // seconds
  double ramp_elapsed_ = 0.0;
  double control_dt_ = 0.005;   // 1/control_rate
  std::vector<double> ramp_start_positions_;
  std::vector<double> ramp_target_positions_;

  // Callbacks
  void joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg);
  void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg);
  void posture_target_cb(const std_msgs::msg::String::SharedPtr msg);
  void com_target_cb(const geometry_msgs::msg::Vector3::SharedPtr msg);
  void control_loop();
};
