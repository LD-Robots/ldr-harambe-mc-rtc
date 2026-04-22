// mc_rtc_bridge_real — bridge for the real Harambe robot
//
// Differences vs mc_rtc_bridge (Gazebo):
//   - Uses per-group FollowJointTrajectory action servers (not whole_body_controller topic)
//   - Handles joint name mapping: mc_rtc uses *_X6/_X4/_X8 suffixes,
//     real robot uses bare names for legs/waist (e.g. "waist_yaw_joint" vs "waist_yaw_joint_X8")
//   - Subscribes to /joint_states (from joint_state_broadcaster, 100Hz)
//   - Sends trajectories to 5 group controllers (left_arm, right_arm, waist, left_leg, right_leg)

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <mc_control/mc_global_controller.h>
#include <mc_tasks/CoMTask.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

using namespace std::chrono_literals;
using FollowJT = control_msgs::action::FollowJointTrajectory;

class McRtcBridgeReal : public rclcpp::Node
{
public:
  McRtcBridgeReal() : Node("mc_rtc_bridge_real")
  {
    declare_parameter("control_rate", 100.0);
    declare_parameter("joint_state_topic", std::string("/joint_states"));
    declare_parameter("imu_topic", std::string("/pelvis_imu/data"));
    declare_parameter("startup_duration", 3.0);
    declare_parameter("trajectory_dt", 0.05);  // command ahead time for JTC

    double rate = get_parameter("control_rate").as_double();
    control_dt_ = 1.0 / rate;
    ramp_duration_ = get_parameter("startup_duration").as_double();
    traj_dt_ = get_parameter("trajectory_dt").as_double();
    auto js_topic = get_parameter("joint_state_topic").as_string();
    auto imu_topic = get_parameter("imu_topic").as_string();

    // --- mc_rtc name -> real robot name ---
    // Arms keep their _X6/_X4 suffix. Waist & legs drop the _X8 suffix.
    mcrtc_to_real_name_ = {
      {"waist_yaw_joint_X8", "waist_yaw_joint"},
      {"left_hip_pitch_joint_X8", "left_hip_pitch_joint"},
      {"left_hip_roll_joint_X8", "left_hip_roll_joint"},
      {"left_hip_yaw_joint_X8", "left_hip_yaw_joint"},
      {"left_knee_joint_X8", "left_knee_joint"},
      {"left_ankle_pitch_joint_X4", "left_ankle_pitch_joint"},
      {"left_ankle_roll_joint_X4", "left_ankle_roll_joint"},
      {"right_hip_pitch_joint_X8", "right_hip_pitch_joint"},
      {"right_hip_roll_joint_X8", "right_hip_roll_joint"},
      {"right_hip_yaw_joint_X8", "right_hip_yaw_joint"},
      {"right_knee_joint_X8", "right_knee_joint"},
      {"right_ankle_pitch_joint_X4", "right_ankle_pitch_joint"},
      {"right_ankle_roll_joint_X4", "right_ankle_roll_joint"},
      // Arms: identity mapping (names already match)
    };
    // Build reverse map (real -> mc_rtc) for joint_states parsing
    for (const auto & [mc, real] : mcrtc_to_real_name_)
      real_to_mcrtc_name_[real] = mc;

    // --- Groups (each has its own FollowJointTrajectory action) ---
    groups_ = {
      {"left_arm_group_controller",
        {"left_shoulder_pitch_joint_X6", "left_shoulder_roll_joint_X6",
         "left_shoulder_yaw_joint_X4", "left_elbow_pitch_joint_X6",
         "left_wrist_yaw_joint_X4", "left_wrist_roll_joint_X4"}},
      {"right_arm_group_controller",
        {"right_shoulder_pitch_joint_X6", "right_shoulder_roll_joint_X6",
         "right_shoulder_yaw_joint_X4", "right_elbow_pitch_joint_X6",
         "right_wrist_yaw_joint_X4", "right_wrist_roll_joint_X4"}},
      {"waist_controller",
        {"waist_yaw_joint_X8"}},
      {"left_leg_group_controller",
        {"left_hip_pitch_joint_X8", "left_hip_roll_joint_X8",
         "left_hip_yaw_joint_X8", "left_knee_joint_X8",
         "left_ankle_pitch_joint_X4", "left_ankle_roll_joint_X4"}},
      {"right_leg_group_controller",
        {"right_hip_pitch_joint_X8", "right_hip_roll_joint_X8",
         "right_hip_yaw_joint_X8", "right_knee_joint_X8",
         "right_ankle_roll_joint_X4"}},  // pitch missing on real right ankle
    };

    // --- mc_rtc controller ---
    gc_ = std::make_unique<mc_control::MCGlobalController>();
    RCLCPP_INFO(get_logger(), "mc_rtc controller: %s", gc_->current_controller().c_str());
    RCLCPP_INFO(get_logger(), "mc_rtc timestep: %.4f s", gc_->timestep());
    ref_joint_order_ = gc_->ref_joint_order();
    RCLCPP_INFO(get_logger(), "mc_rtc ref_joint_order: %zu joints", ref_joint_order_.size());

    encoder_positions_.resize(ref_joint_order_.size(), 0.0);
    encoder_velocities_.resize(ref_joint_order_.size(), 0.0);

    // --- Subscribers ---
    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      js_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) { joint_state_cb(msg); });

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) { imu_cb(msg); });

    posture_sub_ = create_subscription<std_msgs::msg::String>(
      "~/posture_target", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) { posture_target_cb(msg); });

    com_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
      "~/com_target", 10,
      [this](const geometry_msgs::msg::Vector3::SharedPtr msg) { com_target_cb(msg); });

    // --- Action clients per group ---
    for (const auto & g : groups_)
    {
      auto client = rclcpp_action::create_client<FollowJT>(
        this, "/" + g.first + "/follow_joint_trajectory");
      action_clients_[g.first] = client;
      RCLCPP_INFO(get_logger(), "Created action client for /%s/follow_joint_trajectory", g.first.c_str());
    }

    // --- Control timer ---
    auto period = std::chrono::duration<double>(control_dt_);
    timer_ = create_wall_timer(period, [this]() { control_loop(); });

    RCLCPP_INFO(get_logger(), "mc_rtc_bridge_real ready. Rate=%.0f Hz, listening on %s", rate, js_topic.c_str());
  }

private:
  // --- Callbacks ---
  void joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    // Build name->idx map from first message
    if (!js_received_)
    {
      js_name_to_idx_.clear();
      for (size_t i = 0; i < msg->name.size(); ++i)
        js_name_to_idx_[msg->name[i]] = i;

      size_t found = 0;
      for (size_t i = 0; i < ref_joint_order_.size(); ++i)
      {
        const std::string mc_name = ref_joint_order_[i];
        // Try real-robot name first, then mc_rtc name (arms)
        std::string lookup_name = mc_name;
        auto it = mcrtc_to_real_name_.find(mc_name);
        if (it != mcrtc_to_real_name_.end())
          lookup_name = it->second;
        if (js_name_to_idx_.count(lookup_name))
          found++;
        else
          RCLCPP_WARN(get_logger(), "mc_rtc joint '%s' (real: '%s') not in joint_states",
                      mc_name.c_str(), lookup_name.c_str());
      }
      RCLCPP_INFO(get_logger(), "Joint mapping: %zu/%zu mc_rtc joints found in joint_states",
                  found, ref_joint_order_.size());
      js_received_ = true;
    }

    // Update encoder arrays
    for (size_t i = 0; i < ref_joint_order_.size(); ++i)
    {
      std::string lookup_name = ref_joint_order_[i];
      auto mit = mcrtc_to_real_name_.find(ref_joint_order_[i]);
      if (mit != mcrtc_to_real_name_.end())
        lookup_name = mit->second;

      auto it = js_name_to_idx_.find(lookup_name);
      if (it != js_name_to_idx_.end())
      {
        size_t idx = it->second;
        if (idx < msg->position.size()) encoder_positions_[i] = msg->position[idx];
        if (idx < msg->velocity.size()) encoder_velocities_[i] = msg->velocity[idx];
      }
    }
  }

  void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    imu_orientation_ = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x,
                                          msg->orientation.y, msg->orientation.z);
    imu_angular_velocity_ = Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y,
                                            msg->angular_velocity.z);
    imu_received_ = true;
  }

  void posture_target_cb(const std_msgs::msg::String::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    std::string data = msg->data;
    size_t start = 0;
    while (start < data.size())
    {
      size_t end = data.find(';', start);
      if (end == std::string::npos) end = data.size();
      std::string pair = data.substr(start, end - start);
      size_t colon = pair.find(':');
      if (colon != std::string::npos)
      {
        std::string joint_name = pair.substr(0, colon);
        try
        {
          double value = std::stod(pair.substr(colon + 1));
          pending_posture_targets_[joint_name] = {value};
        }
        catch (...) {}
      }
      start = end + 1;
    }
  }

  void com_target_cb(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (initialized_)
    {
      Eigen::Vector3d current_com = gc_->controller().robot().com();
      pending_com_target_ = current_com + Eigen::Vector3d(msg->x, msg->y, msg->z);
      com_target_pending_ = true;
      RCLCPP_INFO(get_logger(), "CoM target: current=[%.3f,%.3f,%.3f] offset=[%.3f,%.3f,%.3f]",
                  current_com.x(), current_com.y(), current_com.z(), msg->x, msg->y, msg->z);
    }
  }

  // --- Control loop ---
  void control_loop()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!js_received_) return;

    // Ramp startup: hold current positions (no trajectory during ramp)
    if (!initialized_ && !ramping_)
    {
      RCLCPP_INFO(get_logger(), "Starting stance ramp over %.1f seconds...", ramp_duration_);
      ramp_start_positions_ = encoder_positions_;
      ramp_target_positions_.resize(ref_joint_order_.size(), 0.0);
      const auto & stance = gc_->robot().stance();
      for (size_t i = 0; i < ref_joint_order_.size(); ++i)
      {
        auto it = stance.find(ref_joint_order_[i]);
        if (it != stance.end() && !it->second.empty())
          ramp_target_positions_[i] = it->second[0];
        else
          ramp_target_positions_[i] = ramp_start_positions_[i];
      }
      ramp_elapsed_ = 0.0;
      ramping_ = true;
      return;
    }

    if (ramping_)
    {
      ramp_elapsed_ += control_dt_;
      double alpha = std::min(ramp_elapsed_ / ramp_duration_, 1.0);
      double t = alpha * alpha * (3.0 - 2.0 * alpha);
      std::vector<double> cmd(ref_joint_order_.size());
      for (size_t i = 0; i < ref_joint_order_.size(); ++i)
        cmd[i] = ramp_start_positions_[i] + t * (ramp_target_positions_[i] - ramp_start_positions_[i]);
      send_trajectories(cmd);

      if (alpha >= 1.0)
      {
        RCLCPP_INFO(get_logger(), "Ramp complete. Initializing mc_rtc...");
        ramping_ = false;
        gc_->init(ramp_target_positions_);
        gc_->running = true;
        initialized_ = true;
      }
      return;
    }

    // Normal operation: feed mc_rtc, run, publish
    gc_->setEncoderValues(encoder_positions_);
    gc_->setEncoderVelocities(encoder_velocities_);
    if (imu_received_)
    {
      gc_->setSensorOrientation(imu_orientation_);
      gc_->setSensorAngularVelocity(imu_angular_velocity_);
    }

    if (!pending_posture_targets_.empty())
    {
      auto posture_task = gc_->controller().postureTask;
      if (posture_task) posture_task->target(pending_posture_targets_);
      pending_posture_targets_.clear();
    }

    if (com_target_pending_)
    {
      for (auto * task : gc_->controller().solver().tasks())
      {
        auto * com_task = dynamic_cast<mc_tasks::CoMTask *>(task);
        if (com_task) { com_task->com(pending_com_target_); break; }
      }
      com_target_pending_ = false;
    }

    if (!gc_->run())
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "mc_rtc run() returned false");
      return;
    }

    const auto & robot = gc_->controller().outputRobot();
    const auto & q = robot.q();
    std::vector<double> cmd(ref_joint_order_.size(), 0.0);
    for (size_t i = 0; i < ref_joint_order_.size(); ++i)
    {
      int mbc_idx = robot.jointIndexInMBC(i);
      if (mbc_idx >= 0 && static_cast<size_t>(mbc_idx) < q.size() && !q[mbc_idx].empty())
        cmd[i] = q[mbc_idx][0];
      else
        cmd[i] = encoder_positions_[i];
    }
    send_trajectories(cmd);
  }

  // --- Send FollowJointTrajectory goals to each group ---
  void send_trajectories(const std::vector<double> & cmd_by_mcrtc_index)
  {
    for (const auto & g : groups_)
    {
      const std::string & group_name = g.first;
      const auto & joint_names = g.second;
      auto client = action_clients_[group_name];
      if (!client->action_server_is_ready()) continue;

      trajectory_msgs::msg::JointTrajectory traj;
      for (const auto & mc_name : joint_names)
      {
        std::string real_name = mc_name;
        auto it = mcrtc_to_real_name_.find(mc_name);
        if (it != mcrtc_to_real_name_.end()) real_name = it->second;
        traj.joint_names.push_back(real_name);
      }

      trajectory_msgs::msg::JointTrajectoryPoint pt;
      for (const auto & mc_name : joint_names)
      {
        // Find mc_name index in ref_joint_order
        auto it = std::find(ref_joint_order_.begin(), ref_joint_order_.end(), mc_name);
        if (it != ref_joint_order_.end())
        {
          size_t idx = std::distance(ref_joint_order_.begin(), it);
          pt.positions.push_back(cmd_by_mcrtc_index[idx]);
        }
        else
        {
          pt.positions.push_back(0.0);
        }
      }
      pt.time_from_start = rclcpp::Duration::from_seconds(traj_dt_);
      traj.points.push_back(pt);

      FollowJT::Goal goal;
      goal.trajectory = traj;
      client->async_send_goal(goal);
    }
  }

  // --- Members ---
  std::unique_ptr<mc_control::MCGlobalController> gc_;
  std::vector<std::string> ref_joint_order_;
  std::vector<double> encoder_positions_, encoder_velocities_;

  std::map<std::string, std::string> mcrtc_to_real_name_;
  std::map<std::string, std::string> real_to_mcrtc_name_;
  std::vector<std::pair<std::string, std::vector<std::string>>> groups_;
  std::map<std::string, rclcpp_action::Client<FollowJT>::SharedPtr> action_clients_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr posture_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr com_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  // State
  std::mutex state_mutex_;
  std::map<std::string, size_t> js_name_to_idx_;
  bool js_received_ = false;
  bool imu_received_ = false;
  bool initialized_ = false;
  bool ramping_ = false;

  Eigen::Quaterniond imu_orientation_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d imu_angular_velocity_ = Eigen::Vector3d::Zero();

  std::map<std::string, std::vector<double>> pending_posture_targets_;
  Eigen::Vector3d pending_com_target_ = Eigen::Vector3d::Zero();
  bool com_target_pending_ = false;

  std::vector<double> ramp_start_positions_, ramp_target_positions_;
  double ramp_elapsed_ = 0.0;
  double ramp_duration_ = 3.0;
  double control_dt_ = 0.01;
  double traj_dt_ = 0.05;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<McRtcBridgeReal>());
  rclcpp::shutdown();
  return 0;
}
