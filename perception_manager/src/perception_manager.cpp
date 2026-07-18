#include <algorithm>
#include <memory>
#include <random>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "perception_interfaces/msg/detection.hpp"

using namespace std::chrono_literals;

class PerceptionManager : public rclcpp::Node
{
public:
  using Detection = perception_interfaces::msg::Detection;

  PerceptionManager()
  : rclcpp::Node("perception_manager"),
    rng_(std::random_device{}())
  {
    // ---- Parameter: minimum detection confidence, range [0.0, 1.0] ------
    this->declare_parameter<double>("confidence_threshold", 0.5);

    // Reject out-of-range values no matter how they are set
    // (YAML, `ros2 param set`, or — later — the service).
    param_cb_handle_ = this->add_on_set_parameters_callback(
      [](const std::vector<rclcpp::Parameter> & params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & p : params) {
          if (p.get_name() == "confidence_threshold") {
            const double v = p.as_double();
            if (v < 0.0 || v > 1.0) {
              result.successful = false;
              result.reason = "confidence_threshold must be in [0.0, 1.0]";
            }
          }
        }
        return result;
      });

    // ---- Publisher -------------------------------------------------------
    detection_pub_ = this->create_publisher<Detection>("detections", rclcpp::QoS(10));

    // ---- TEMPORARY: 10 Hz dummy publisher (replaced by the action server)
    timer_ = this->create_wall_timer(100ms, [this]() {
      detection_pub_->publish(makeDetection("test_object"));
    });

    RCLCPP_INFO(this->get_logger(),
                "perception_manager ready (confidence_threshold = %.2f)",
                this->get_parameter("confidence_threshold").as_double());
  }

private:
  /// Build one random detection honouring the CURRENT threshold, so later
  /// runtime changes take effect immediately.
  Detection makeDetection(const std::string & target_class)
  {
    const double threshold = this->get_parameter("confidence_threshold").as_double();

    std::uniform_real_distribution<float> conf_dist(static_cast<float>(threshold), 1.0f);
    std::uniform_real_distribution<float> x_dist(0.0f, 640.0f);
    std::uniform_real_distribution<float> y_dist(0.0f, 480.0f);

    // Two samples per axis, sorted -> always a valid box.
    float x1 = x_dist(rng_), x2 = x_dist(rng_);
    float y1 = y_dist(rng_), y2 = y_dist(rng_);

    Detection msg;
    msg.class_name = target_class;
    msg.confidence = conf_dist(rng_);
    msg.x_min = std::min(x1, x2);
    msg.x_max = std::max(x1, x2);
    msg.y_min = std::min(y1, y2);
    msg.y_max = std::max(y1, y2);
    msg.stamp = this->now();
    return msg;
  }

  rclcpp::Publisher<Detection>::SharedPtr detection_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  std::mt19937 rng_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionManager>());
  rclcpp::shutdown();
  return 0;
}