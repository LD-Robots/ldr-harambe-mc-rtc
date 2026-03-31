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

# --- ONNX Walking Controller ---
# ONNX Runtime from ldr-harambe-arms-sim2real/third_party/onnxruntime
set(ONNXRT_DIR "$ENV{HOME}/Documents/GitHub/ldr-harambe-arms-sim2real/third_party/onnxruntime")
if(EXISTS "${ONNXRT_DIR}/include/onnxruntime_cxx_api.h")
  message(STATUS "Using ONNX Runtime: ${ONNXRT_DIR}")

  add_controller(
    "HarambeOnnxWalking"
    "onnx_walking_controller.cpp"
    "onnx_walking_controller.h"
  )
  target_include_directories(HarambeOnnxWalking PRIVATE "${ONNXRT_DIR}/include")
  target_link_directories(HarambeOnnxWalking PRIVATE "${ONNXRT_DIR}/lib")
  target_link_libraries(HarambeOnnxWalking PRIVATE onnxruntime)
  # Set RPATH so libonnxruntime.so is found at runtime
  set_target_properties(HarambeOnnxWalking PROPERTIES
    BUILD_RPATH "${ONNXRT_DIR}/lib"
    INSTALL_RPATH "${ONNXRT_DIR}/lib"
  )
else()
  message(WARNING "ONNX Runtime not found at ${ONNXRT_DIR}, skipping HarambeOnnxWalking")
endif()
