#pragma once

#include <mc_control/mc_controller.h>
#include <onnxruntime_cxx_api.h>

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace mc_control
{

/// mc_rtc controller that runs an ONNX RL locomotion policy with dual IMU.
/// Bypasses the QP solver entirely — writes joint position targets
/// directly to robot().mbc().q so mc_mujoco applies its PD control.
///
/// Observation (93 dims):
///   base_lin_vel(3) +
///   pelvis_imu_ang_vel(3) + pelvis_imu_gravity(3) +
///   torso_imu_ang_vel(3) + torso_imu_gravity(3) +
///   commands(3) + joint_pos_rel(25) + joint_vel(25) + prev_action(25)
class HarambeOnnxWalkingController : public MCController
{
public:
  HarambeOnnxWalkingController(mc_rbdyn::RobotModulePtr rm, double dt, Backend backend);

  bool run() override;
  void reset(const ControllerResetData & reset_data) override;

private:
  static constexpr int NUM_JOINTS = 25;
  static constexpr int OBS_DIM = 93;

  // --- ONNX runtime ---
  std::unique_ptr<Ort::Env> ortEnv_;
  std::unique_ptr<Ort::Session> ortSession_;
  Ort::MemoryInfo ortMemInfo_{Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)};

  // --- Policy state ---
  std::vector<float> obs_;            // 93-dim observation
  std::array<float, NUM_JOINTS> prevAction_{};
  std::array<double, NUM_JOINTS> targetPos_{};
  std::array<double, NUM_JOINTS> defaultPos_{};

  // --- Joint mapping ---
  std::array<int, NUM_JOINTS> refToMbc_{};

  // --- Effort limits (for target clamping) ---
  std::array<double, NUM_JOINTS> effortLimit_{};
  std::array<double, NUM_JOINTS> kp_{};

  // --- Parameters ---
  double actionScale_ = 0.25;
  double clipActions_ = 5.0;
  double smoothing_ = 1.0;
  int decimation_ = 4;
  int warmupSteps_ = 0;
  int stepCounter_ = 0;
  int policyStep_ = 0;
  bool policyActive_ = false;

  // --- Velocity commands ---
  Eigen::Vector3d commands_ = Eigen::Vector3d::Zero();

  // --- Dual IMU orientation estimation ---
  Eigen::Quaterniond pelvisOrientation_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond torsoOrientation_ = Eigen::Quaterniond::Identity();
  double dt_ = 0.005;

  // --- Methods ---
  void buildObservation();
  void runInference();
  void loadConfig(const mc_rtc::Configuration & cfg);

  /// Complementary filter: integrate gyro + correct with accelerometer
  void updateOrientation(Eigen::Quaterniond & orientation,
                         const Eigen::Vector3d & angVel,
                         const Eigen::Vector3d & accel);
};

}  // namespace mc_control
