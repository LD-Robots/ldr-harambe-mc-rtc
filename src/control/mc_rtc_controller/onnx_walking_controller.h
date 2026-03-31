#pragma once

#include <mc_control/mc_controller.h>
#include <onnxruntime_cxx_api.h>

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace mc_control
{

/// mc_rtc controller that runs an ONNX RL locomotion policy.
/// Bypasses the QP solver entirely — writes joint position targets
/// directly to robot().mbc().q so mc_mujoco applies its PD control.
class HarambeOnnxWalkingController : public MCController
{
public:
  HarambeOnnxWalkingController(mc_rbdyn::RobotModulePtr rm, double dt, Backend backend);

  bool run() override;
  void reset(const ControllerResetData & reset_data) override;

private:
  static constexpr int NUM_JOINTS = 25;
  static constexpr int OBS_DIM = 87;

  // --- ONNX runtime ---
  std::unique_ptr<Ort::Env> ortEnv_;
  std::unique_ptr<Ort::Session> ortSession_;
  Ort::MemoryInfo ortMemInfo_{Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)};

  // --- Policy state ---
  std::vector<float> obs_;            // 87-dim observation
  std::array<float, NUM_JOINTS> prevAction_{};
  std::array<double, NUM_JOINTS> targetPos_{};
  std::array<double, NUM_JOINTS> defaultPos_{};

  // --- Joint mapping ---
  // Maps policy index (0..24, same as ref_joint_order) to mbc joint index
  std::array<int, NUM_JOINTS> refToMbc_{};

  // --- Parameters ---
  double actionScale_ = 0.25;
  double clipActions_ = 5.0;
  int decimation_ = 4;
  int warmupSteps_ = 0;
  int stepCounter_ = 0;
  int policyStep_ = 0;
  bool policyActive_ = false;

  // --- Velocity commands ---
  Eigen::Vector3d commands_ = Eigen::Vector3d::Zero();

  // --- IMU sensor name ---
  std::string imuSensor_ = "FloatingBase";

  // --- Orientation estimation (complementary filter) ---
  Eigen::Quaterniond baseOrientation_ = Eigen::Quaterniond::Identity();
  double dt_ = 0.005;

  // --- Methods ---
  void buildObservation();
  void runInference();
  void loadConfig(const mc_rtc::Configuration & cfg);
};

}  // namespace mc_control
