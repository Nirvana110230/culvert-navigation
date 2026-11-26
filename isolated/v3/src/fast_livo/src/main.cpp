#include "LIVMapper.h"
#include "session_gravity.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto nh = std::make_shared<rclcpp::Node>("fastlivo_transport");
  image_transport::ImageTransport it_(nh);
  try {
    LIVMapper mapper(nh, "laserMapping");
    mapper.initializeSubscribersAndPublishers(nh, it_);
    mapper.run(nh);
  } catch (const std::invalid_argument& error) {
    RCLCPP_ERROR(nh->get_logger(), "%s", error.what());
    rclcpp::shutdown();
    return 2;
  } catch (const session_gravity::Error& error) {
    RCLCPP_ERROR(nh->get_logger(), "%s", error.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}
