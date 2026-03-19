#pragma once

#include <mc_control/fsm/Controller.h>

namespace mc_control
{

struct HarambeWalkingController : public fsm::Controller
{
  HarambeWalkingController(const mc_rbdyn::RobotModulePtr & rm, double dt,
                           const Configuration & config)
  : fsm::Controller(rm, dt, config)
  {
  }
};

} // namespace mc_control
