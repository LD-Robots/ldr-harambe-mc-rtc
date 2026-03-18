cmake_minimum_required(VERSION 3.8)
project(mc_rtc_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# mc_rtc headers via pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(mc_rtc REQUIRED IMPORTED_TARGET mc_rtc)

# Collect all mc_rtc shared libraries from /usr/local/lib
# (colcon cmake isolation prevents find_package(mc_rtc) from resolving targets)
set(MC_RTC_LIBS
  /usr/local/lib/libmc_control.so
  /usr/local/lib/libmc_rbdyn.so
  /usr/local/lib/libmc_rtc_gui.so
  /usr/local/lib/libmc_rtc_utils.so
  /usr/local/lib/libmc_rtc_loader.so
  /usr/local/lib/libmc_solver.so
  /usr/local/lib/libmc_tasks.so
  /usr/local/lib/libmc_trajectory.so
  /usr/local/lib/libmc_planning.so
  /usr/local/lib/libTVM.so
  /usr/local/lib/libRBDyn.so
  /usr/local/lib/libRBDynParsers.so
  /usr/local/lib/libTasks.so
  /usr/local/lib/libstate-observation.so
  /usr/local/lib/libeigen-quadprog.so
  boost_stacktrace_backtrace
  boost_filesystem
  fmt
  spdlog
  yaml-cpp
  ltdl
)

# Library (reusable from other packages)
add_library(mc_rtc_bridge_lib SHARED src/mc_rtc_bridge.cpp)

target_include_directories(mc_rtc_bridge_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${mc_rtc_INCLUDE_DIRS}
  /usr/local/include/tvm/3rd-party
)

target_link_libraries(mc_rtc_bridge_lib
  ${MC_RTC_LIBS}
  Eigen3::Eigen
)

ament_target_dependencies(mc_rtc_bridge_lib
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
)

# Executable (standalone node)
add_executable(mc_rtc_bridge src/mc_rtc_bridge_node.cpp)

target_link_libraries(mc_rtc_bridge
  mc_rtc_bridge_lib
)

install(TARGETS mc_rtc_bridge_lib
  EXPORT mc_rtc_bridge_libExport
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS mc_rtc_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(mc_rtc_bridge_lib)
ament_export_dependencies(rclcpp sensor_msgs std_msgs geometry_msgs)

ament_package()
