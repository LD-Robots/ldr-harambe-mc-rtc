#include "harambe_walking_controller.h"
#include <mc_control/mc_controller.h>

MULTI_CONTROLLERS_CONSTRUCTOR("HarambeWalking",
                              mc_control::HarambeWalkingController(rm, dt, config),
                              "HarambeWalking_TVM",
                              mc_control::HarambeWalkingController(rm, dt, config))
