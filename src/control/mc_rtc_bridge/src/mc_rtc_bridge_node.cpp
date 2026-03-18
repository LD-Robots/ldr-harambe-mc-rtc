#include "mc_rtc_bridge/mc_rtc_bridge.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<McRtcBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
