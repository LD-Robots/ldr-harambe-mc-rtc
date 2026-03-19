cmake_minimum_required(VERSION 3.10)
project(mc_rtc_controller)

# Install locally in the build tree instead of /usr/local
set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}" CACHE PATH "" FORCE)
set(MC_RTC_HONOR_INSTALL_PREFIX ON)

find_package(mc_rtc REQUIRED)

add_controller(
  "HarambeCoMPosture"
  "mc_com_posture_controller.cpp"
  "mc_com_posture_controller.h"
)

add_controller(
  "HarambeWalking"
  "harambe_walking_controller.cpp"
  "harambe_walking_controller.h"
)
target_link_libraries(HarambeWalking PUBLIC mc_rtc::mc_control_fsm)
