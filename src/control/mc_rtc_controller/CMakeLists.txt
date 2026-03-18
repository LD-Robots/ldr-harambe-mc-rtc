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
