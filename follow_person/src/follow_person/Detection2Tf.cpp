#include "follow_person/Detection2Tf.hpp"
#include <tf2_ros/transform_broadcaster.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace follow_person
{
using std::placeholders::_1;
using namespace std::chrono_literals;

Detection2Tf::Detection2Tf()
: Node ("yolo_3d_to_tf"),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  yolo_sub_ = create_subscription<yolo_msgs::msg::DetectionArray>(
    "/yolo/detections", 10, std::bind(&Detection2Tf::generate_tf, this, _1));
  timer_ = create_wall_timer(
    500ms, std::bind(&Detection2Tf::publish_tf, this));
}

void
Detection2Tf::generate_tf(const yolo_msgs::msg::DetectionArray last_msg)
{

  if (last_msg.detections.empty()) {
    RCLCPP_WARN(this->get_logger(), "No detections available.");
    return;
  }
  
  if (last_msg.header.frame_id.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Error: last_msg.header.frame_id is empty!");
    return;
  }

  for (const auto & detection : last_msg.detections) {
    if (detection.class_name == "person") {
      auto position = detection.bbox3d.center.position;

      transform_.header.frame_id = "_rgb_optical_frame";
      transform_.child_frame_id = "target";

      if (!std::isnan(position.x) && !std::isinf(position.x) && !std::isnan(position.y) && !std::isinf(position.y) && !std::isnan(position.z) && !std::isinf(position.z)) {
        transform_.transform.translation.x = position.x;
        transform_.transform.translation.y = position.y;
        transform_.transform.translation.z = position.z;
        RCLCPP_INFO(this->get_logger(), "Generated TF: [%.2f, %.2f, %.2f]",
                    position.x, position.y, position.z);
      } else {
        RCLCPP_INFO(get_logger(), "Error in Detection");
      }
      // Tomamos la primera persona que detecte
      break;
    }
  }

}

void
Detection2Tf::publish_tf()
{
  transform_.header.stamp = now();
  tf_broadcaster_->sendTransform(transform_);
}

}