
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

class CameraTfBroadcaster : public rclcpp::Node
{
public:
  CameraTfBroadcaster()
  : rclcpp::Node("camera_tf_broadcaster")
  {
    // Extrinsic parameters (defaults = identity transform)
    this->declare_parameter("tx", 0.0);
    this->declare_parameter("ty", 0.0);
    this->declare_parameter("tz", 0.0);
    this->declare_parameter("roll", 0.0);
    this->declare_parameter("pitch", 0.0);
    this->declare_parameter("yaw", 0.0);

    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    broadcaster_->sendTransform(buildTransform());
  }

private:
  geometry_msgs::msg::TransformStamped buildTransform()
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();
    t.header.frame_id = "base_link";
    t.child_frame_id = "camera_optical_frame";

    t.transform.translation.x = this->get_parameter("tx").as_double();
    t.transform.translation.y = this->get_parameter("ty").as_double();
    t.transform.translation.z = this->get_parameter("tz").as_double();

    // The TF message stores orientation as a quaternion; convert from RPY.
    tf2::Quaternion q;
    q.setRPY(this->get_parameter("roll").as_double(),
             this->get_parameter("pitch").as_double(),
             this->get_parameter("yaw").as_double());
    q.normalize();
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    RCLCPP_INFO(this->get_logger(),
                "Static TF base_link -> camera_optical_frame published "
                "(t=[%.3f %.3f %.3f] m)",
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z);
    return t;
  }

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // Keep spinning so the latched transform stays available to late joiners.
  rclcpp::spin(std::make_shared<CameraTfBroadcaster>());
  rclcpp::shutdown();
  return 0;
}