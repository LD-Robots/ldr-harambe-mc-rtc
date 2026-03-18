#include "mc_rtc_bridge/mc_rtc_bridge.hpp"

McRtcBridge::McRtcBridge()
: Node("mc_rtc_bridge")
{
  // Parameters
  declare_parameter("control_rate", 200.0);
  declare_parameter("joint_state_topic", std::string("/joint_states"));
  declare_parameter("command_topic", std::string("/whole_body_controller/commands"));
  declare_parameter("imu_topic", std::string("/pelvis_imu/data"));
  declare_parameter("passthrough_joints", std::vector<std::string>{});
  declare_parameter("startup_duration", 3.0);

  double rate = get_parameter("control_rate").as_double();
  ramp_duration_ = get_parameter("startup_duration").as_double();
  control_dt_ = 1.0 / rate;
  auto js_topic = get_parameter("joint_state_topic").as_string();
  auto cmd_topic = get_parameter("command_topic").as_string();
  auto imu_topic = get_parameter("imu_topic").as_string();
  passthrough_joints_ = get_parameter("passthrough_joints").as_string_array();

  // Initialize mc_rtc MCGlobalController (reads ~/.config/mc_rtc/mc_rtc.yaml)
  gc_ = std::make_unique<mc_control::MCGlobalController>();
  RCLCPP_INFO(get_logger(), "mc_rtc controller: %s", gc_->current_controller().c_str());
  RCLCPP_INFO(get_logger(), "mc_rtc timestep: %.4f s", gc_->timestep());

  // Get ref_joint_order from mc_rtc (the order it expects encoder data)
  ref_joint_order_ = gc_->ref_joint_order();
  RCLCPP_INFO(get_logger(), "mc_rtc ref_joint_order: %zu joints", ref_joint_order_.size());
  for (size_t i = 0; i < ref_joint_order_.size(); ++i)
  {
    RCLCPP_DEBUG(get_logger(), "  [%zu] %s", i, ref_joint_order_[i].c_str());
  }

  // Subscriber: /joint_states
  js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    js_topic, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) { joint_state_cb(msg); });

  // Subscriber: IMU
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::Imu::SharedPtr msg) { imu_cb(msg); });

  // Subscriber: posture target (format: "joint_name:value" or "joint1:val1;joint2:val2")
  posture_sub_ = create_subscription<std_msgs::msg::String>(
    "~/posture_target", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) { posture_target_cb(msg); });

  // Subscriber: CoM target offset (Vector3: x,y,z offset from current CoM in meters)
  com_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
    "~/com_target", 10,
    [this](const geometry_msgs::msg::Vector3::SharedPtr msg) { com_target_cb(msg); });

  // Publisher: /whole_body_controller/commands
  cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(cmd_topic, 10);

  // Timer: control loop
  auto period = std::chrono::duration<double>(1.0 / rate);
  timer_ = create_wall_timer(period, [this]() { control_loop(); });

  RCLCPP_INFO(get_logger(), "mc_rtc_bridge ready. Rate=%.0f Hz, listening on %s, publishing to %s",
              rate, js_topic.c_str(), cmd_topic.c_str());
}

// --- Public API ---

void McRtcBridge::setPostureTarget(const std::map<std::string, std::vector<double>> & targets)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (const auto & [name, val] : targets)
  {
    pending_posture_targets_[name] = val;
  }
}

void McRtcBridge::setComOffset(const Eigen::Vector3d & offset)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (initialized_)
  {
    Eigen::Vector3d current_com = gc_->controller().robot().com();
    pending_com_target_ = current_com + offset;
    com_target_pending_ = true;
  }
}

bool McRtcBridge::isInitialized() const
{
  return initialized_;
}

const std::vector<std::string> & McRtcBridge::refJointOrder() const
{
  return ref_joint_order_;
}

// --- Callbacks ---

void McRtcBridge::joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  // First message: build name mapping
  if (!js_received_)
  {
    js_name_to_idx_.clear();
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
      js_name_to_idx_[msg->name[i]] = i;
    }

    // Build wbc_joint_order from joint_states (this IS the whole_body_controller order)
    wbc_joint_order_ = msg->name;

    // Allocate encoder arrays
    encoder_positions_.resize(ref_joint_order_.size(), 0.0);
    encoder_velocities_.resize(ref_joint_order_.size(), 0.0);

    // Build mc_rtc -> wbc mapping
    mcrtc_to_wbc_.resize(ref_joint_order_.size(), -1);
    for (size_t i = 0; i < ref_joint_order_.size(); ++i)
    {
      for (size_t j = 0; j < wbc_joint_order_.size(); ++j)
      {
        if (ref_joint_order_[i] == wbc_joint_order_[j])
        {
          mcrtc_to_wbc_[i] = static_cast<int>(j);
          break;
        }
      }
      if (mcrtc_to_wbc_[i] < 0)
      {
        RCLCPP_WARN(get_logger(), "mc_rtc joint '%s' not found in joint_states!",
                    ref_joint_order_[i].c_str());
      }
    }

    // Identify passthrough joints in wbc order
    for (size_t j = 0; j < wbc_joint_order_.size(); ++j)
    {
      bool is_mcrtc = false;
      for (const auto & name : ref_joint_order_)
      {
        if (name == wbc_joint_order_[j])
        {
          is_mcrtc = true;
          break;
        }
      }
      if (!is_mcrtc)
      {
        passthrough_positions_[j] = 0.0;
      }
    }

    RCLCPP_INFO(get_logger(), "Joint mapping built: %zu mc_rtc joints, %zu wbc joints, %zu passthrough",
                ref_joint_order_.size(), wbc_joint_order_.size(), passthrough_positions_.size());

    js_received_ = true;
  }

  // Update encoder values in ref_joint_order
  for (size_t i = 0; i < ref_joint_order_.size(); ++i)
  {
    auto it = js_name_to_idx_.find(ref_joint_order_[i]);
    if (it != js_name_to_idx_.end())
    {
      size_t idx = it->second;
      if (idx < msg->position.size())
        encoder_positions_[i] = msg->position[idx];
      if (idx < msg->velocity.size())
        encoder_velocities_[i] = msg->velocity[idx];
    }
  }

  // Update passthrough joint positions
  for (auto & [wbc_idx, pos] : passthrough_positions_)
  {
    if (wbc_idx < msg->position.size())
      pos = msg->position[wbc_idx];
  }
}

void McRtcBridge::imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  imu_orientation_ = Eigen::Quaterniond(
    msg->orientation.w, msg->orientation.x,
    msg->orientation.y, msg->orientation.z);
  imu_angular_velocity_ = Eigen::Vector3d(
    msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  imu_linear_acceleration_ = Eigen::Vector3d(
    msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  imu_received_ = true;
}

void McRtcBridge::posture_target_cb(const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  // Parse "joint_name:value" pairs separated by ";"
  std::string data = msg->data;
  size_t start = 0;
  while (start < data.size())
  {
    size_t end = data.find(';', start);
    if (end == std::string::npos)
      end = data.size();

    std::string pair = data.substr(start, end - start);
    size_t colon = pair.find(':');
    if (colon != std::string::npos)
    {
      std::string joint_name = pair.substr(0, colon);
      try
      {
        double value = std::stod(pair.substr(colon + 1));
        pending_posture_targets_[joint_name] = {value};
        RCLCPP_INFO(get_logger(), "Posture target: %s = %.4f", joint_name.c_str(), value);
      }
      catch (const std::exception & e)
      {
        RCLCPP_WARN(get_logger(), "Invalid posture target value: '%s'", pair.c_str());
      }
    }
    start = end + 1;
  }
}

void McRtcBridge::com_target_cb(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (initialized_)
  {
    Eigen::Vector3d current_com = gc_->controller().robot().com();
    pending_com_target_ = current_com + Eigen::Vector3d(msg->x, msg->y, msg->z);
    com_target_pending_ = true;
    RCLCPP_INFO(get_logger(), "CoM target: current=[%.3f, %.3f, %.3f] + offset=[%.3f, %.3f, %.3f] = [%.3f, %.3f, %.3f]",
                current_com.x(), current_com.y(), current_com.z(),
                msg->x, msg->y, msg->z,
                pending_com_target_.x(), pending_com_target_.y(), pending_com_target_.z());
  }
}

void McRtcBridge::control_loop()
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (!js_received_)
    return;

  // Start ramp phase on first joint state
  if (!initialized_ && !ramping_)
  {
    RCLCPP_INFO(get_logger(), "Starting stance ramp over %.1f seconds...", ramp_duration_);

    // Save current positions as ramp start
    ramp_start_positions_ = encoder_positions_;

    // Build ramp target from stance
    const auto & stance = gc_->robot().stance();
    ramp_target_positions_.resize(ref_joint_order_.size(), 0.0);
    for (size_t i = 0; i < ref_joint_order_.size(); ++i)
    {
      auto it = stance.find(ref_joint_order_[i]);
      if (it != stance.end() && !it->second.empty())
      {
        ramp_target_positions_[i] = it->second[0];
        RCLCPP_INFO(get_logger(), "  ramp %s: %.4f -> %.4f",
                    ref_joint_order_[i].c_str(), ramp_start_positions_[i], ramp_target_positions_[i]);
      }
      else
      {
        ramp_target_positions_[i] = ramp_start_positions_[i];
      }
    }

    ramp_elapsed_ = 0.0;
    ramping_ = true;
    return;
  }

  // Ramp phase: interpolate from current to stance
  if (ramping_)
  {
    ramp_elapsed_ += control_dt_;
    double alpha = std::min(ramp_elapsed_ / ramp_duration_, 1.0);
    // Smooth cubic interpolation (ease in/out)
    double t = alpha * alpha * (3.0 - 2.0 * alpha);

    std_msgs::msg::Float64MultiArray cmd_msg;
    cmd_msg.data.resize(ref_joint_order_.size());
    for (size_t i = 0; i < ref_joint_order_.size(); ++i)
    {
      cmd_msg.data[i] = ramp_start_positions_[i] + t * (ramp_target_positions_[i] - ramp_start_positions_[i]);
    }
    cmd_pub_->publish(cmd_msg);

    if (alpha >= 1.0)
    {
      RCLCPP_INFO(get_logger(), "Ramp complete. Initializing mc_rtc...");
      ramping_ = false;

      // Init mc_rtc with stance positions (robot is now at stance)
      gc_->init(ramp_target_positions_);
      gc_->running = true;
      initialized_ = true;
      RCLCPP_INFO(get_logger(), "mc_rtc initialized and running!");
    }
    return;
  }

  // 1. Feed encoder data to mc_rtc
  gc_->setEncoderValues(encoder_positions_);
  gc_->setEncoderVelocities(encoder_velocities_);

  // 2. Feed IMU data if available
  if (imu_received_)
  {
    gc_->setSensorOrientation(imu_orientation_);
    gc_->setSensorAngularVelocity(imu_angular_velocity_);
  }

  // 3. Apply pending posture targets
  if (!pending_posture_targets_.empty())
  {
    auto posture_task = gc_->controller().postureTask;
    if (posture_task)
    {
      posture_task->target(pending_posture_targets_);
    }
    pending_posture_targets_.clear();
  }

  // 4. Apply pending CoM target
  if (com_target_pending_)
  {
    for (auto * task : gc_->controller().solver().tasks())
    {
      auto * com_task = dynamic_cast<mc_tasks::CoMTask *>(task);
      if (com_task)
      {
        com_task->com(pending_com_target_);
        RCLCPP_INFO_ONCE(get_logger(), "CoMTask found and target set");
        break;
      }
    }
    com_target_pending_ = false;
  }

  // 5. Run one mc_rtc control step
  if (!gc_->run())
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "mc_rtc run() returned false");
    return;
  }

  // 6. Read target joint positions from mc_rtc output robot (after QP solve)
  const auto & robot = gc_->controller().outputRobot();
  const auto & q = robot.q();

  // 7. Build command message: 25 values in ref_joint_order
  std_msgs::msg::Float64MultiArray cmd_msg;
  cmd_msg.data.resize(ref_joint_order_.size(), 0.0);

  for (size_t i = 0; i < ref_joint_order_.size(); ++i)
  {
    int mbc_idx = robot.jointIndexInMBC(i);
    if (mbc_idx >= 0 && static_cast<size_t>(mbc_idx) < q.size() && !q[mbc_idx].empty())
    {
      cmd_msg.data[i] = q[mbc_idx][0];
    }
    else
    {
      cmd_msg.data[i] = encoder_positions_[i];
    }
  }

  // Log first few commands for debugging
  static int log_count = 0;
  if (log_count < 3)
  {
    std::string vals;
    for (size_t i = 0; i < std::min(cmd_msg.data.size(), size_t(6)); ++i)
      vals += " " + std::to_string(cmd_msg.data[i]);
    RCLCPP_INFO(get_logger(), "CMD[%d] first 6:%s (total %zu)", log_count, vals.c_str(), cmd_msg.data.size());
    log_count++;
  }

  // 8. Publish
  cmd_pub_->publish(cmd_msg);
}
