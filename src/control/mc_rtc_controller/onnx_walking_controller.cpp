#include "onnx_walking_controller.h"

#include <mc_rtc/logging.h>
#include <mc_rtc/path.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <filesystem>

namespace mc_control
{

// ============================================================================
// Constructor — minimal: no QP constraints, no tasks
// ============================================================================
HarambeOnnxWalkingController::HarambeOnnxWalkingController(std::shared_ptr<mc_rbdyn::RobotModule> rm,
                                                           double dt,
                                                           Backend backend)
: MCController(rm, dt, backend)
{
  // Do NOT add any QP constraints or tasks — we bypass the solver entirely.
  obs_.resize(OBS_DIM, 0.0f);
  prevAction_.fill(0.0f);
  mc_rtc::log::success("[OnnxWalking] Constructor done (no QP tasks/constraints)");
}

// ============================================================================
// reset — load config, build joint mapping, load ONNX model
// ============================================================================
void HarambeOnnxWalkingController::reset(const ControllerResetData & reset_data)
{
  MCController::reset(reset_data);
  robot().forwardKinematics();
  robot().forwardVelocity();

  // --- Load configuration ---
  mc_rtc::Configuration cfg = config();
  // mc_rtc may not merge controller YAML into config(); load manually
  // mc_rtc may not merge controller YAML; always try loading manually
  {
    const auto userControllers = mc_rtc::user_config_directory_path("controllers");
    std::filesystem::path cfgPath = std::filesystem::path(userControllers) / (name_ + ".yaml");
    if(!std::filesystem::exists(cfgPath))
    {
      cfgPath = std::filesystem::path(userControllers) / "HarambeOnnxWalking.yaml";
    }
    if(std::filesystem::exists(cfgPath))
    {
      mc_rtc::log::info("[OnnxWalking] Loading config from {}", cfgPath.string());
      cfg.load(cfgPath.string());
    }
  }
  mc_rtc::log::info("[OnnxWalking] Config keys: {}", fmt::join(cfg.keys(), ", "));
  loadConfig(cfg);

  // --- Build ref_joint_order -> mbc index mapping ---
  const auto & rjo = robot().module().ref_joint_order();
  refToMbc_.fill(-1);
  for(int i = 0; i < NUM_JOINTS && i < static_cast<int>(rjo.size()); i++)
  {
    if(robot().hasJoint(rjo[i]))
    {
      int mbcIdx = robot().jointIndexByName(rjo[i]);
      refToMbc_[i] = mbcIdx;
    }
    else
    {
      mc_rtc::log::warning("[OnnxWalking] Joint '{}' not found in robot", rjo[i]);
    }
  }

  // --- Default positions from stance ---
  const auto & stance = robot().module().stance();
  for(int i = 0; i < NUM_JOINTS && i < static_cast<int>(rjo.size()); i++)
  {
    auto it = stance.find(rjo[i]);
    if(it != stance.end() && !it->second.empty())
    {
      defaultPos_[i] = it->second[0];
    }
    else
    {
      defaultPos_[i] = 0.0;
    }
  }

  // --- Effort limits and PD gains per joint (matching Gazebo plugin / Isaac Lab) ---
  {
    // Effort limits from actuator specs
    const std::map<std::string, double> effortMap = {
      {"shoulder_pitch", 23.0}, {"shoulder_roll", 23.0}, {"shoulder_yaw", 12.075},
      {"elbow_pitch", 23.0}, {"wrist_yaw", 12.075}, {"wrist_roll", 12.075},
      {"waist_yaw", 72.0},
      {"hip_pitch", 72.0}, {"hip_roll", 72.0}, {"hip_yaw", 72.0},
      {"knee", 72.0},
      {"ankle_pitch", 96.6}, {"ankle_roll", 28.98},
    };
    // kp from PDgains (read at mc_mujoco level, we duplicate here for clamp calc)
    const std::map<std::string, double> kpMap = {
      {"shoulder_pitch", 114.3}, {"shoulder_roll", 115.4}, {"shoulder_yaw", 22.0},
      {"elbow_pitch", 30.6}, {"wrist_yaw", 9.1}, {"wrist_roll", 10.8},
      {"waist_yaw", 91.7},
      {"hip_pitch", 466.2}, {"hip_roll", 464.4}, {"hip_yaw", 75.7},
      {"knee", 97.3},
      {"ankle_pitch", 996.2}, {"ankle_roll", 89.7},
    };
    for(int i = 0; i < NUM_JOINTS && i < static_cast<int>(rjo.size()); i++)
    {
      effortLimit_[i] = 100.0;  // default
      kp_[i] = 200.0;           // default
      for(const auto & [key, val] : effortMap)
      {
        if(rjo[i].find(key) != std::string::npos) { effortLimit_[i] = val; break; }
      }
      for(const auto & [key, val] : kpMap)
      {
        if(rjo[i].find(key) != std::string::npos) { kp_[i] = val; break; }
      }
    }
  }

  // --- Set initial targets to default positions ---
  for(int i = 0; i < NUM_JOINTS; i++)
  {
    targetPos_[i] = defaultPos_[i];
    int mbcIdx = refToMbc_[i];
    if(mbcIdx >= 0 && mbcIdx < static_cast<int>(robot().mbc().q.size())
       && !robot().mbc().q[mbcIdx].empty())
    {
      robot().mbc().q[mbcIdx][0] = defaultPos_[i];
    }
  }
  robot().forwardKinematics();

  // --- Log encoder state at reset ---
  {
    const auto & ev = realRobot().encoderValues();
    mc_rtc::log::info("[OnnxWalking] encoderValues at reset: size={}", ev.size());
    if(ev.size() >= 25)
    {
      mc_rtc::log::info("[OnnxWalking]   ev[0:6]=[{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}]",
                        ev[0], ev[1], ev[2], ev[3], ev[4], ev[5]);
      mc_rtc::log::info("[OnnxWalking]   ev[13:19]=[{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}]",
                        ev[13], ev[14], ev[15], ev[16], ev[17], ev[18]);
    }
  }

  // --- Reset state ---
  prevAction_.fill(0.0f);
  std::fill(obs_.begin(), obs_.end(), 0.0f);
  stepCounter_ = 0;
  policyStep_ = 0;
  policyActive_ = false;
  pelvisOrientation_ = Eigen::Quaterniond::Identity();
  torsoOrientation_ = Eigen::Quaterniond::Identity();
  dt_ = timeStep;

  // --- Add GUI elements ---
  gui()->addElement({"OnnxWalking"},
    mc_rtc::gui::NumberInput("cmd_vx", [this]() { return commands_.x(); },
                             [this](double v) { commands_.x() = v; }),
    mc_rtc::gui::NumberInput("cmd_vy", [this]() { return commands_.y(); },
                             [this](double v) { commands_.y() = v; }),
    mc_rtc::gui::NumberInput("cmd_yaw", [this]() { return commands_.z(); },
                             [this](double v) { commands_.z() = v; }),
    mc_rtc::gui::NumberInput("action_scale", [this]() { return actionScale_; },
                             [this](double v) { actionScale_ = v; }),
    mc_rtc::gui::NumberInput("smoothing", [this]() { return smoothing_; },
                             [this](double v) { smoothing_ = std::clamp(v, 0.01, 1.0); }),
    mc_rtc::gui::NumberInput("clip_actions", [this]() { return clipActions_; },
                             [this](double v) { clipActions_ = std::max(v, 0.1); })
  );

  // --- Add logging ---
  logger().addLogEntry("OnnxWalking_cmd_vx", [this]() { return commands_.x(); });
  logger().addLogEntry("OnnxWalking_cmd_vy", [this]() { return commands_.y(); });
  logger().addLogEntry("OnnxWalking_cmd_yaw", [this]() { return commands_.z(); });
  logger().addLogEntry("OnnxWalking_policy_active", [this]() -> double { return policyActive_ ? 1.0 : 0.0; });
  logger().addLogEntry("OnnxWalking_step", [this]() -> double { return stepCounter_; });

  mc_rtc::log::success("[OnnxWalking] Reset complete. decimation={}, action_scale={}, warmup={}",
                       decimation_, actionScale_, warmupSteps_);
  mc_rtc::log::info("[OnnxWalking] Commands: vx={}, vy={}, yaw={}", commands_.x(), commands_.y(), commands_.z());
  mc_rtc::log::info("[OnnxWalking] ONNX loaded: {}", ortSession_ != nullptr);
}

// ============================================================================
// run — bypass QP, write joint targets directly
// ============================================================================
bool HarambeOnnxWalkingController::run()
{
  // Do NOT call MCController::run() — that would invoke the QP solver.
  // But we DO need the observer pipeline to update realRobot state (orientation, etc.)
  runObserverPipelines();

  // 1. Build observation from realRobot (sensor data from mc_mujoco)
  buildObservation();

  // 2. Policy inference at decimated rate
  if(stepCounter_ % decimation_ == 0)
  {
    if(stepCounter_ >= warmupSteps_ * decimation_)
    {
      runInference();
      policyActive_ = true;
    }
    policyStep_++;
  }

  // 3. Write targets to robot().mbc().q
  for(int i = 0; i < NUM_JOINTS; i++)
  {
    int mbcIdx = refToMbc_[i];
    if(mbcIdx >= 0 && mbcIdx < static_cast<int>(robot().mbc().q.size())
       && !robot().mbc().q[mbcIdx].empty())
    {
      robot().mbc().q[mbcIdx][0] = targetPos_[i];
    }
  }

  // 4. Update FK for consistency (GUI display, logging, etc.)
  robot().forwardKinematics();
  robot().forwardVelocity();

  // 5. Periodic logging (also log first active step)
  if(stepCounter_ % 1000 == 1 || (policyActive_ && stepCounter_ == warmupSteps_ * decimation_ + 1))
  {
    mc_rtc::log::info("[OnnxWalking] step={}, policy_active={}, policy_step={}",
                      stepCounter_, policyActive_, policyStep_);
    // Full obs dump (same format as Isaac Lab FULL_OBS_93)
    std::string obsStr = "FULL_OBS_93:";
    for(int i = 0; i < OBS_DIM; i++)
    {
      if(i > 0) obsStr += ",";
      obsStr += fmt::format("{:.6f}", obs_[i]);
    }
    mc_rtc::log::info("[OnnxWalking]   {}", obsStr);
    mc_rtc::log::info("[OnnxWalking]   base_lin_vel:   [{:.3f},{:.3f},{:.3f}]", obs_[0], obs_[1], obs_[2]);
    mc_rtc::log::info("[OnnxWalking]   pelvis_ang_vel: [{:.3f},{:.3f},{:.3f}]", obs_[3], obs_[4], obs_[5]);
    mc_rtc::log::info("[OnnxWalking]   pelvis_gravity: [{:.3f},{:.3f},{:.3f}]", obs_[6], obs_[7], obs_[8]);
    mc_rtc::log::info("[OnnxWalking]   torso_ang_vel:  [{:.3f},{:.3f},{:.3f}]", obs_[9], obs_[10], obs_[11]);
    mc_rtc::log::info("[OnnxWalking]   torso_gravity:  [{:.3f},{:.3f},{:.3f}]", obs_[12], obs_[13], obs_[14]);
    mc_rtc::log::info("[OnnxWalking]   commands:       [{:.3f},{:.3f},{:.3f}]", obs_[15], obs_[16], obs_[17]);
    mc_rtc::log::info("[OnnxWalking]   pos_rel[0:6]:   [{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}] (left arm)",
                      obs_[18], obs_[19], obs_[20], obs_[21], obs_[22], obs_[23]);
    mc_rtc::log::info("[OnnxWalking]   pos_rel[13:19]: [{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}] (left leg)",
                      obs_[31], obs_[32], obs_[33], obs_[34], obs_[35], obs_[36]);
    mc_rtc::log::info("[OnnxWalking]   pos_rel[19:25]: [{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}] (right leg)",
                      obs_[37], obs_[38], obs_[39], obs_[40], obs_[41], obs_[42]);
    mc_rtc::log::info("[OnnxWalking]   action[13..18]: [{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}]",
                      prevAction_[13], prevAction_[14], prevAction_[15],
                      prevAction_[16], prevAction_[17], prevAction_[18]);
  }

  stepCounter_++;
  return true;
}

// ============================================================================
// updateOrientation — complementary filter for one IMU
// ============================================================================
void HarambeOnnxWalkingController::updateOrientation(Eigen::Quaterniond & orientation,
                                                      const Eigen::Vector3d & angVel,
                                                      const Eigen::Vector3d & accel)
{
  // Gyro integration
  Eigen::Vector3d dtheta = angVel * dt_;
  double angle = dtheta.norm();
  if(angle > 1e-10)
  {
    Eigen::Quaterniond dq(Eigen::AngleAxisd(angle, dtheta / angle));
    orientation = (orientation * dq).normalized();
  }

  // Accelerometer correction
  double accelNorm = accel.norm();
  if(accelNorm > 5.0 && accelNorm < 15.0)
  {
    Eigen::Vector3d measGrav = -accel / accelNorm;
    Eigen::Matrix3d R = orientation.toRotationMatrix();
    Eigen::Vector3d estGrav = R.transpose() * Eigen::Vector3d(0, 0, -1);
    Eigen::Vector3d error = measGrav.cross(estGrav);
    double alpha = 0.02;
    Eigen::Quaterniond correction(1.0, alpha * error.x() * 0.5,
                                       alpha * error.y() * 0.5,
                                       alpha * error.z() * 0.5);
    orientation = (correction * orientation).normalized();
  }
}

// ============================================================================
// buildObservation — 93-dim vector with dual IMU
// Layout: base_lin_vel(3) +
//         pelvis_ang_vel(3) + pelvis_gravity(3) +
//         torso_ang_vel(3) + torso_gravity(3) +
//         commands(3) + joint_pos_rel(25) + joint_vel(25) + prev_action(25)
// ============================================================================
void HarambeOnnxWalkingController::buildObservation()
{
  const auto & real = realRobot();
  const auto & rjo = robot().module().ref_joint_order();
  const auto & ev = real.encoderValues();
  const auto & evd = real.encoderVelocities();

  // --- Pelvis IMU (on urdf_base) ---
  Eigen::Vector3d pelvisAngVel = Eigen::Vector3d::Zero();
  Eigen::Vector3d pelvisGravity = Eigen::Vector3d(0.0, 0.0, -1.0);
  if(real.hasBodySensor("PelvisIMU"))
  {
    const auto & bs = real.bodySensor("PelvisIMU");
    pelvisAngVel = bs.angularVelocity();
    updateOrientation(pelvisOrientation_, pelvisAngVel, bs.linearAcceleration());
    pelvisGravity = pelvisOrientation_.toRotationMatrix().transpose() * Eigen::Vector3d(0, 0, -1);
  }

  // --- Torso IMU (on urdf_simplified_torso) ---
  Eigen::Vector3d torsoAngVel = Eigen::Vector3d::Zero();
  Eigen::Vector3d torsoGravity = Eigen::Vector3d(0.0, 0.0, -1.0);
  if(real.hasBodySensor("TorsoIMU"))
  {
    const auto & bs = real.bodySensor("TorsoIMU");
    torsoAngVel = bs.angularVelocity();
    updateOrientation(torsoOrientation_, torsoAngVel, bs.linearAcceleration());
    torsoGravity = torsoOrientation_.toRotationMatrix().transpose() * Eigen::Vector3d(0, 0, -1);
  }

  int idx = 0;

  // base_lin_vel (3) — zero
  obs_[idx++] = 0.0f; obs_[idx++] = 0.0f; obs_[idx++] = 0.0f;

  // pelvis_imu_ang_vel (3)
  obs_[idx++] = static_cast<float>(pelvisAngVel.x());
  obs_[idx++] = static_cast<float>(pelvisAngVel.y());
  obs_[idx++] = static_cast<float>(pelvisAngVel.z());

  // pelvis_imu_gravity (3)
  obs_[idx++] = static_cast<float>(pelvisGravity.x());
  obs_[idx++] = static_cast<float>(pelvisGravity.y());
  obs_[idx++] = static_cast<float>(pelvisGravity.z());

  // torso_imu_ang_vel (3)
  obs_[idx++] = static_cast<float>(torsoAngVel.x());
  obs_[idx++] = static_cast<float>(torsoAngVel.y());
  obs_[idx++] = static_cast<float>(torsoAngVel.z());

  // torso_imu_gravity (3)
  obs_[idx++] = static_cast<float>(torsoGravity.x());
  obs_[idx++] = static_cast<float>(torsoGravity.y());
  obs_[idx++] = static_cast<float>(torsoGravity.z());

  // commands (3)
  obs_[idx++] = static_cast<float>(commands_.x());
  obs_[idx++] = static_cast<float>(commands_.y());
  obs_[idx++] = static_cast<float>(commands_.z());

  // joint_pos_rel (25)
  for(int i = 0; i < NUM_JOINTS && i < static_cast<int>(rjo.size()); i++)
  {
    if(i < static_cast<int>(ev.size()))
    {
      obs_[idx + i] = static_cast<float>(ev[i] - defaultPos_[i]);
    }
  }
  idx += NUM_JOINTS;

  // joint_vel (25)
  for(int i = 0; i < NUM_JOINTS && i < static_cast<int>(rjo.size()); i++)
  {
    if(i < static_cast<int>(evd.size()))
    {
      obs_[idx + i] = static_cast<float>(evd[i]);
    }
  }
  idx += NUM_JOINTS;

  // prev_action (25)
  for(int i = 0; i < NUM_JOINTS; i++)
  {
    obs_[idx + i] = prevAction_[i];
  }
}

// ============================================================================
// runInference — ONNX policy forward pass
// ============================================================================
void HarambeOnnxWalkingController::runInference()
{
  if(!ortSession_) return;

  std::array<int64_t, 2> inputShape = {1, OBS_DIM};
  auto inputTensor = Ort::Value::CreateTensor<float>(
      ortMemInfo_, obs_.data(), obs_.size(),
      inputShape.data(), inputShape.size());

  const char * inputNames[] = {"obs"};
  const char * outputNames[] = {"actions"};

  auto outputTensors = ortSession_->Run(
      Ort::RunOptions{nullptr},
      inputNames, &inputTensor, 1,
      outputNames, 1);

  float * actions = outputTensors[0].GetTensorMutableData<float>();

  for(int i = 0; i < NUM_JOINTS; i++)
  {
    float a = actions[i];
    a = std::clamp(a, static_cast<float>(-clipActions_), static_cast<float>(clipActions_));
    prevAction_[i] = a;
    double rawTarget = a * actionScale_ + defaultPos_[i];
    // Low-pass filter: target = smoothing * new + (1 - smoothing) * prev
    double smoothed = smoothing_ * rawTarget + (1.0 - smoothing_) * targetPos_[i];
    // Effort-based target clamp: limit how far target can be from current position
    // so that PD torque doesn't exceed effort_limit: |kp * (target - pos)| <= effort_limit
    // max_error = effort_limit / kp
    double maxError = effortLimit_[i] / kp_[i];
    // Read current position from encoder
    const auto & ev = realRobot().encoderValues();
    if(i < static_cast<int>(ev.size()))
    {
      double currentPos = ev[i];
      smoothed = std::clamp(smoothed, currentPos - maxError, currentPos + maxError);
    }
    targetPos_[i] = smoothed;
  }
}

// ============================================================================
// loadConfig — read YAML configuration
// ============================================================================
void HarambeOnnxWalkingController::loadConfig(const mc_rtc::Configuration & cfg)
{
  // --- ONNX model ---
  std::string modelPath;
  if(cfg.has("onnx"))
  {
    auto onnxCfg = cfg("onnx");
    onnxCfg("model_path", modelPath);
    onnxCfg("action_scale", actionScale_);
    onnxCfg("clip_actions", clipActions_);
    onnxCfg("decimation", decimation_);
    onnxCfg("warmup_steps", warmupSteps_);
    onnxCfg("smoothing", smoothing_);
    mc_rtc::log::info("[OnnxWalking] onnx config keys: {}", fmt::join(onnxCfg.keys(), ", "));
    mc_rtc::log::info("[OnnxWalking] Read: action_scale={}, clip={}, decimation={}, warmup={}",
                      actionScale_, clipActions_, decimation_, warmupSteps_);
  }

  // --- Load ONNX model ---
  if(!modelPath.empty())
  {
    // Expand ~ to HOME
    if(modelPath.front() == '~')
    {
      const char * home = std::getenv("HOME");
      if(home) modelPath = std::string(home) + modelPath.substr(1);
    }
    if(std::filesystem::exists(modelPath))
    {
      try
      {
        ortEnv_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "HarambeOnnxWalking");
        Ort::SessionOptions opts;
        opts.SetIntraOpNumThreads(1);
        ortSession_ = std::make_unique<Ort::Session>(*ortEnv_, modelPath.c_str(), opts);
        mc_rtc::log::success("[OnnxWalking] ONNX model loaded from {}", modelPath);
      }
      catch(const Ort::Exception & e)
      {
        mc_rtc::log::error("[OnnxWalking] Failed to load ONNX model: {}", e.what());
      }
    }
    else
    {
      mc_rtc::log::error("[OnnxWalking] ONNX model not found: {}", modelPath);
    }
  }
  else
  {
    mc_rtc::log::warning("[OnnxWalking] No onnx.model_path specified, policy will output zeros");
  }

  // --- Commands ---
  if(cfg.has("commands"))
  {
    auto cmdCfg = cfg("commands");
    double vx = 0, vy = 0, yaw = 0;
    cmdCfg("vx", vx);
    cmdCfg("vy", vy);
    cmdCfg("yaw_rate", yaw);
    commands_ = Eigen::Vector3d(vx, vy, yaw);
  }

  // IMU sensors are hardcoded: PelvisIMU + TorsoIMU
}

}  // namespace mc_control

MULTI_CONTROLLERS_CONSTRUCTOR("HarambeOnnxWalking",
                              mc_control::HarambeOnnxWalkingController(rm, dt, mc_control::MCController::Backend::Tasks),
                              "HarambeOnnxWalking",
                              mc_control::HarambeOnnxWalkingController(rm, dt, mc_control::MCController::Backend::TVM))
