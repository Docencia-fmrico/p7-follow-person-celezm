#include "follow_person/Detection2Tf.hpp"
#include <tf2_ros/transform_broadcaster.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include "vision_msgs/msg/detection3_d_array.hpp"

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
  detection_sub_ = create_subscription<vision_msgs::msg::Detection3DArray>(
    "/detections_3d", 10, std::bind(&Detection2Tf::generate_tf, this, _1));
  /*detection_sub_ = create_subscription<vision_msgs::msg::Detection3DArray>(
    "/input_detections", 10, std::bind(&Detection2Tf::generate_tf, this, _1));*/
  timer_ = create_wall_timer(
    500ms, std::bind(&Detection2Tf::publish_tf, this));

  last_transform_.transform.translation.x = 0.0;
  last_transform_.transform.translation.y = 0.0;
  last_transform_.transform.translation.z = 0.0;
}

void
Detection2Tf::generate_tf(const vision_msgs::msg::Detection3DArray last_msg)
{

  if (last_msg.detections.empty()) {
    RCLCPP_WARN(this->get_logger(), "No detections available.");
    transform_.transform.translation.x = 0.0;
    transform_.transform.translation.y = 0.0;
    transform_.transform.translation.z = 0.0;
    transform_.header.frame_id = "base_footprint";
    transform_.child_frame_id = "target";
    return;
  }
  
  if (last_msg.header.frame_id.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Error: last_msg.header.frame_id is empty!");
    transform_.transform.translation.x = 0.0;
    transform_.transform.translation.y = 0.0;
    transform_.transform.translation.z = 0.0;
    transform_.header.frame_id = "base_footprint";
    transform_.child_frame_id = "target";
    return;
  }

  auto detection = last_msg.detections[0];

  RCLCPP_INFO(this->get_logger(), "Entro en el callback");


  tf2::Transform bf2opt, opt2target;
  geometry_msgs::msg::TransformStamped opt2target_msg;
  std::string error;

  // for (const auto & detection : last_msg.detections) {
  // Obtenemos la primera persona que se detecte:
  auto position = detection.bbox.center.position;

  //opt2target_msg.header.frame_id = detection.header.frame_id;
  opt2target_msg.header.frame_id = "camera_link";
  opt2target_msg.child_frame_id = "target";

  RCLCPP_INFO(get_logger(), "Recibo detección de %s", detection.header.frame_id.c_str());

  //RCLCPP_INFO(this->get_logger(), "Recibo detección de %s", detection.header.frame_id.c_str());

  if (!std::isnan(position.x) && !std::isinf(position.x) && !std::isnan(position.y) && !std::isinf(position.y) && !std::isnan(position.z) && !std::isinf(position.z)) {
    opt2target_msg.transform.translation.x = position.z;
    opt2target_msg.transform.translation.y = -1 * position.x;
    opt2target_msg.transform.translation.z = -1 * position.y;
    RCLCPP_INFO(this->get_logger(), "Generated TF 1: [%.2f, %.2f, %.2f]",
                position.x, position.y, position.z);
  } else {
    RCLCPP_INFO(get_logger(), "Error in Detection");
  }

  RCLCPP_INFO(get_logger(), "Antes del if de la tf");

  if (tf_buffer_.canTransform("base_footprint", "camera_link", tf2::TimePointZero, &error)) {
    RCLCPP_INFO(get_logger(), "Antes de obtener la tf");
    auto bf2opt_msg = tf_buffer_.lookupTransform(
      "base_footprint", "camera_link", tf2::TimePointZero);

    RCLCPP_INFO(get_logger(), "Obtengo TF");
    
    tf2::fromMsg(bf2opt_msg.transform, bf2opt);
    tf2::fromMsg(opt2target_msg.transform, opt2target);

    RCLCPP_INFO(get_logger(), "Calculo de la que publico");

    auto tf_bf2tar = bf2opt * opt2target;

    transform_.header.frame_id = "base_footprint";
    transform_.child_frame_id = "target";
    transform_.transform = tf2::toMsg(tf_bf2tar);

    RCLCPP_INFO(this->get_logger(), "Generated TF 2: [%.2f, %.2f, %.2f]",
                transform_.transform.translation.x, transform_.transform.translation.y, 
                transform_.transform.translation.z);
  } else {
    opt2target_msg.transform.translation.x = 0.0;
    opt2target_msg.transform.translation.y = 0.0;
    opt2target_msg.transform.translation.z = 0.0;
  }

  //break;
  //}

}

void
Detection2Tf::publish_tf()
{  
  if (transform_.header.frame_id.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Error: last_msg.header.frame_id is empty!");
    transform_.transform.translation.x = 0.0;
    transform_.transform.translation.y = 0.0;
    transform_.transform.translation.z = 0.0;
    transform_.header.frame_id = "base_footprint";
    transform_.child_frame_id = "target";
    return;
  }

  if (transform_.transform.translation.x == last_transform_.transform.translation.x &&
    transform_.transform.translation.y == last_transform_.transform.translation.y &&
    transform_.transform.translation.z == last_transform_.transform.translation.z) {

    transform_.transform.translation.x = 0.0;
    transform_.transform.translation.y = 0.0;
    transform_.transform.translation.z = 0.0;

  } else {
    last_transform_.transform.translation.x = transform_.transform.translation.x;
    last_transform_.transform.translation.y = transform_.transform.translation.y;
    last_transform_.transform.translation.z = transform_.transform.translation.z;
  }

  transform_.header.stamp = now();
  RCLCPP_INFO(get_logger(), "Antes de publicar");
  tf_broadcaster_->sendTransform(transform_);
}

}