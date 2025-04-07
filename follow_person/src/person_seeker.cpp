#include "follow_person/FollowPersonNode.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto avoidance_node = std::make_shared<follow_person
  ::FollowPersonNode>();
  
  rclcpp::spin(avoidance_node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
