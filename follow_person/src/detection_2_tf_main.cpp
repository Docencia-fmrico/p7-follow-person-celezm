#include "follow_person/Detection2Tf.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto avoidance_node = std::make_shared<follow_person
      ::Detection2Tf>();
  rclcpp::spin(avoidance_node);

  rclcpp::shutdown();

  return 0;
}
