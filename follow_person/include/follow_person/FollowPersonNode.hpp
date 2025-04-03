#ifndef FOLLOW_PERSON_NODE_HPP_
#define FOLLOW_PERSON_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/twist.hpp"
#include "yolo_msgs/msg/detection_array.hpp"

namespace follow_person
{

class FollowPersonNode : public rclcpp::Node
{

public:
  FollowPersonNode();

private:
  rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr attractive_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
};

}
#endif  //  FOLLOW_PERSON_NODE_HPP_