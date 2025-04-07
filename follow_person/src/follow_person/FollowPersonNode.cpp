#include "follow_person/FollowPersonNode.hpp"
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

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

FollowPersonNode::FollowPersonNode()
: LifecycleNode ("follow_person"),
  tf_buffer_(),
  tf_listener_(tf_buffer_),
  vlin_pid_(0.0, 1.0, 0.0, 0.7),
  vrot_pid_(0.0, 1.0, 0.3, 1.0)
{
  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  /*timer_ = create_wall_timer(
    500ms, std::bind(&FollowPersonNode::control_cycle, this));*/
}

CallbackReturn
FollowPersonNode::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Configuring...");

  return CallbackReturn::SUCCESS;
}

CallbackReturn
FollowPersonNode::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Activating...");

  timer_ = create_wall_timer(
    100ms, std::bind(&FollowPersonNode::control_cycle, this));

  vel_pub_->on_activate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn
FollowPersonNode::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Deactivating...");

  timer_ = nullptr;
  vel_pub_->on_deactivate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn
FollowPersonNode::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Cleaning Up...");

  return CallbackReturn::SUCCESS;
}

CallbackReturn
FollowPersonNode::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Shutting Down...");

  return CallbackReturn::SUCCESS;
}

CallbackReturn
FollowPersonNode::on_error(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Error State");

  return CallbackReturn::SUCCESS;
}

void
FollowPersonNode::control_cycle()
{
  tf2::Stamped<tf2::Transform> bf2target;
  std::string error;

  if (tf_buffer_.canTransform("base_footprint", "target", tf2::TimePointZero, &error)) {
    auto bf2target_msg = tf_buffer_.lookupTransform(
      "base_footprint", "target", tf2::TimePointZero);

    tf2::fromMsg(bf2target_msg, bf2target);

    double x = bf2target.getOrigin().x();
    double y = bf2target.getOrigin().y();

    double angle = atan2(y, x);
    double dist = sqrt(x * x + y * y);

    double vel_rot = 0.0;
    double vel_lin = 0.0;

    RCLCPP_INFO(get_logger(), "Distancia al target: %f X: %f Y: %f", dist, x, y);

    if (x > 0.4) {
      vel_rot = std::clamp(vrot_pid_.get_output(angle), -1.5, 1.5);
      vel_lin = std::clamp(vlin_pid_.get_output(dist - 1.0), -0.3, 0.3);
    }

    geometry_msgs::msg::Twist vel;
    vel.linear.x = vel_lin;
    vel.angular.z = vel_rot;

    RCLCPP_INFO(get_logger(), "Vlin: %f Vrot: %f", vel_lin, vel_rot);

    if (fabs(angle) < 0.2 && dist <= 1.0) {
      RCLCPP_INFO(get_logger(), "A 1m de mi objetivo");
    }

    if (abs(last_distance_ - dist) > 1.5) {
      vel.linear.x = 0.0;
      vel.angular.z = 0.0;
    }

    vel_pub_->publish(vel);

    last_distance_ = dist;

  } else {
    RCLCPP_WARN_STREAM(get_logger(), "Error in TF odom -> base_footprint [<< " << error << "]");
  }
}


}  //  namespace follow_person