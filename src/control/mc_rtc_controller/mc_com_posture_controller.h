#pragma once

#include <mc_control/MCController.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/OrientationTask.h>
#include <mc_tasks/lipm_stabilizer/StabilizerTask.h>

#include <memory>

namespace mc_control
{

struct HarambeCoMPostureController final : public MCController
{
public:
  HarambeCoMPostureController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, double dt, Backend backend);

  void reset(const ControllerResetData & reset_data) override;
  bool run() override;

private:
  bool firstRun_ = true;
  std::shared_ptr<mc_tasks::CoMTask> comTask_;
  std::shared_ptr<mc_tasks::OrientationTask> torsoOriTask_;
  std::string torsoBodyName_ = "urdf_simplified_torso";
  std::shared_ptr<mc_tasks::lipm_stabilizer::StabilizerTask> stabilizerTask_;
};

} // namespace mc_control
