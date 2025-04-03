#ifndef YOLO_DETECTION_2_TF_HPP_
#define YOLO_DETECTION_2_TF_HPP_

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/twist.hpp"
#include "yolo_msgs/msg/detection_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2_ros/transform_broadcaster.h>

namespace follow_person
{

class Detection2Tf : public rclcpp::Node
{

public:
  Detection2Tf();

private:
  rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr yolo_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::TransformStamped transform_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  void generate_tf(const yolo_msgs::msg::DetectionArray last_msg);
  void publish_tf();
};

}  // namespace follow_person

#endif  //  YOLO_DETECTION_2_TF_HPP_