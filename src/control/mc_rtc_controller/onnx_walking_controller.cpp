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

  // --- Reset state ---
  prevAction_.fill(0.0f);
  std::fill(obs_.begin(), obs_.end(), 0.0f);
  stepCounter_ = 0;
  policyStep_ = 0;
  policyActive_ = false;
  baseOrientation_ = Eigen::Quaterniond::Identity();
  dt_ = timeStep;

  // --- Add GUI elements ---
  gui()->addElement({"OnnxWalking"},
    mc_rtc::gui::NumberInput("cmd_vx", [this]() { return commands_.x(); },
                             [this](double v) { commands_.x() = v; }),
    mc_rtc::gui::NumberInput("cmd_vy", [this]() { return commands_.y(); },
                             [this](double v) { commands_.y() = v; }),
    mc_rtc::gui::NumberInput("cmd_yaw", [this]() { return commands_.z(); },
                             [this](double v) { commands_.z() = v; })
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

  // 5. Periodic logging
  if(stepCounter_ % 1000 == 1)
  {
    mc_rtc::log::info("[OnnxWalking] step={}, policy_active={}, policy_step={}",
                      stepCounter_, policyActive_, policyStep_);
    // Log orientation estimate
    {
      auto rpy = baseOrientation_.toRotationMatrix().eulerAngles(0, 1, 2);
      mc_rtc::log::info("[OnnxWalking]   ori_rpy=[{:.2f},{:.2f},{:.2f}] deg",
                        rpy.x() * 180.0 / M_PI, rpy.y() * 180.0 / M_PI, rpy.z() * 180.0 / M_PI);
    }
    mc_rtc::log::info("[OnnxWalking]   grav=[{:.3f},{:.3f},{:.3f}] cmd=[{:.2f},{:.2f},{:.2f}]",
                      obs_[6], obs_[7], obs_[8],
                      obs_[9], obs_[10], obs_[11]);
    // Log a few leg actions
    mc_rtc::log::info("[OnnxWalking]   action[13..18]=[{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}]",
                      prevAction_[13], prevAction_[14], prevAction_[15],
                      prevAction_[16], prevAction_[17], prevAction_[18]);
  }

  stepCounter_++;
  return true;
}

// ============================================================================
// buildObservation — 87-dim vector matching Isaac Lab training
// ============================================================================
void HarambeOnnxWalkingController::buildObservation()
{
  const auto & real = realRobot();
  const auto & rjo = robot().module().ref_joint_order();
  const auto & ev = real.encoderValues();
  const auto & evd = real.encoderVelocities();

  // --- Orientation estimation: complementary filter (gyro + accel) ---
  // mc_mujoco only provides gyro and accelerometer, NOT orientation.
  // We integrate gyro and correct drift with accelerometer.
  const std::string baseSensor = imuSensor_;
  Eigen::Vector3d angVelBody = Eigen::Vector3d::Zero();
  Eigen::Vector3d accelBody = Eigen::Vector3d::Zero();

  if(real.hasBodySensor(baseSensor))
  {
    angVelBody = real.bodySensor(baseSensor).angularVelocity();
    accelBody = real.bodySensor(baseSensor).linearAcceleration();
  }

  // Gyro integration: q = q * delta_q(omega * dt)
  {
    Eigen::Vector3d dtheta = angVelBody * dt_;
    double angle = dtheta.norm();
    if(angle > 1e-10)
    {
      Eigen::Quaterniond dq(Eigen::AngleAxisd(angle, dtheta / angle));
      baseOrientation_ = (baseOrientation_ * dq).normalized();
    }
  }

  // Accelerometer correction (complementary filter, alpha=0.02)
  // Only correct when acceleration magnitude is close to gravity (not in free fall or impact)
  {
    double accelNorm = accelBody.norm();
    if(accelNorm > 5.0 && accelNorm < 15.0)  // roughly 0.5g to 1.5g
    {
      // Measured gravity direction in body frame (accelerometer points "up")
      Eigen::Vector3d measGravBody = -accelBody / accelNorm;

      // Expected gravity direction from current orientation estimate
      Eigen::Vector3d gravWorld(0.0, 0.0, -1.0);
      Eigen::Matrix3d R = baseOrientation_.toRotationMatrix();
      Eigen::Vector3d estGravBody = R.transpose() * gravWorld;

      // Correction: rotate towards measured gravity
      Eigen::Vector3d error = measGravBody.cross(estGravBody);
      double alpha = 0.02;  // complementary filter gain
      Eigen::Quaterniond correction(1.0, alpha * error.x() * 0.5,
                                         alpha * error.y() * 0.5,
                                         alpha * error.z() * 0.5);
      baseOrientation_ = (correction * baseOrientation_).normalized();
    }
  }

  Eigen::Matrix3d bodyRot = baseOrientation_.toRotationMatrix();

  // base_lin_vel (3) — zero (not available from IMU alone)
  obs_[0] = obs_[1] = obs_[2] = 0.0f;

  // base_ang_vel (3) — from gyro (already in body frame)
  obs_[3] = static_cast<float>(angVelBody.x());
  obs_[4] = static_cast<float>(angVelBody.y());
  obs_[5] = static_cast<float>(angVelBody.z());

  // projected_gravity (3) — R^T * [0,0,-1]
  {
    Eigen::Vector3d gravWorld(0.0, 0.0, -1.0);
    Eigen::Vector3d gravBody = bodyRot.transpose() * gravWorld;
    obs_[6] = static_cast<float>(gravBody.x());
    obs_[7] = static_cast<float>(gravBody.y());
    obs_[8] = static_cast<float>(gravBody.z());
  }

  // commands (3)
  obs_[9] = static_cast<float>(commands_.x());
  obs_[10] = static_cast<float>(commands_.y());
  obs_[11] = static_cast<float>(commands_.z());

  // joint_pos_rel (25) — encoder position - default position
  for(int i = 0; i < NUM_JOINTS && i < static_cast<int>(rjo.size()); i++)
  {
    int rjoIdx = i;  // ref_joint_order index == policy index
    if(rjoIdx >= 0 && rjoIdx < static_cast<int>(ev.size()))
    {
      obs_[12 + i] = static_cast<float>(ev[rjoIdx] - defaultPos_[i]);
    }
  }

  // joint_vel (25)
  for(int i = 0; i < NUM_JOINTS && i < static_cast<int>(rjo.size()); i++)
  {
    int rjoIdx = i;
    if(rjoIdx >= 0 && rjoIdx < static_cast<int>(evd.size()))
    {
      obs_[37 + i] = static_cast<float>(evd[rjoIdx]);
    }
  }

  // prev_action (25)
  for(int i = 0; i < NUM_JOINTS; i++)
  {
    obs_[62 + i] = prevAction_[i];
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
    targetPos_[i] = a * actionScale_ + defaultPos_[i];
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

  // --- IMU sensor ---
  if(cfg.has("imu_sensor"))
  {
    cfg("imu_sensor", imuSensor_);
  }
}

}  // namespace mc_control

MULTI_CONTROLLERS_CONSTRUCTOR("HarambeOnnxWalking",
                              mc_control::HarambeOnnxWalkingController(rm, dt, mc_control::MCController::Backend::Tasks),
                              "HarambeOnnxWalking",
                              mc_control::HarambeOnnxWalkingController(rm, dt, mc_control::MCController::Backend::TVM))
