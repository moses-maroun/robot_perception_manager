#include <algorithm>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "perception_interfaces/action/start_detection.hpp"
#include "perception_interfaces/msg/detection.hpp"
#include "perception_interfaces/srv/set_confidence_threshold.hpp"

using namespace std::chrono_literals;

class PerceptionManager : public rclcpp::Node
{
public:
  using StartDetection = perception_interfaces::action::StartDetection;
  using GoalHandle = rclcpp_action::ServerGoalHandle<StartDetection>;
  using SetConfidence = perception_interfaces::srv::SetConfidenceThreshold;
  using Detection = perception_interfaces::msg::Detection;

  PerceptionManager()
  : rclcpp::Node("perception_manager"),
    rng_(std::random_device{}())
  {
    //Parameter
    this->declare_parameter<double>("confidence_threshold", 0.5);

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

    //Publisher
    detection_pub_ = this->create_publisher<Detection>("detections", rclcpp::QoS(10));

    // Service server
    set_confidence_srv_ = this->create_service<SetConfidence>(
      "set_confidence",
      std::bind(&PerceptionManager::onSetConfidence, this,
                std::placeholders::_1, std::placeholders::_2));

    // Action server 
    action_server_ = rclcpp_action::create_server<StartDetection>(
      this, "start_detection",
      std::bind(&PerceptionManager::onGoal, this,
                std::placeholders::_1, std::placeholders::_2),
      std::bind(&PerceptionManager::onCancel, this, std::placeholders::_1),
      std::bind(&PerceptionManager::onAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "perception_manager ready (confidence_threshold = %.2f)",
                this->get_parameter("confidence_threshold").as_double());
  }

private:
  // Service: set_confidence 
  void onSetConfidence(const SetConfidence::Request::SharedPtr request,
                       SetConfidence::Response::SharedPtr response)
  {
    const float t = request->threshold;
    if (t < 0.0f || t > 1.0f) {
      response->success = false;
      response->message = "Rejected: threshold " + std::to_string(t) +
                          " is outside the valid range [0.0, 1.0].";
      RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
      return;
    }
    // Route through set_parameter so the parameter server, the validation
    // callback and `ros2 param get` all stay consistent.
    this->set_parameter(rclcpp::Parameter("confidence_threshold",
                                          static_cast<double>(t)));
    response->success = true;
    response->message = "confidence_threshold updated to " + std::to_string(t);
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  }

  // Action: start_detection

  rclcpp_action::GoalResponse onGoal(const rclcpp_action::GoalUUID &,
                                     StartDetection::Goal::ConstSharedPtr goal)
  {
    if (goal->target_class.empty()) {
      RCLCPP_WARN(this->get_logger(), "Rejecting goal with empty target_class");
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(this->get_logger(), "Goal received: target_class='%s'",
                goal->target_class.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse onCancel(const std::shared_ptr<GoalHandle> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Cancel request for goal '%s' -> accepted",
                goal_handle->get_goal()->target_class.c_str());
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void onAccepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    // One worker thread per goal; it blocks until it is its turn to run.
    std::thread(&PerceptionManager::execute, this, goal_handle).detach();
  }

  /// Detection loop for one goal. Runs in its own thread.
  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    // Wait for exclusive ownership of the pipeline.
    {
      std::unique_lock<std::mutex> lock(goal_mutex_);
      goal_done_cv_.wait(lock, [this] { return active_goal_ == nullptr; });
      active_goal_ = goal_handle;
    }

    const std::string target = goal_handle->get_goal()->target_class;
    auto feedback = std::make_shared<StartDetection::Feedback>();
    auto result = std::make_shared<StartDetection::Result>();

    RCLCPP_INFO(this->get_logger(), "Detection started for '%s'", target.c_str());

    int detections = 0;
    const auto t_start = this->now();
    rclcpp::Rate rate(10.0);  // ~10 Hz detections

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        result->total_detections = detections;
        result->success = false;
        goal_handle->canceled(result);
        finishGoal(target, "cancelled");
        return;
      }

      detection_pub_->publish(makeDetection(target));
      ++detections;

      // Feedback every 2nd cycle -> every 0.2 s (~5 Hz).
      if (detections % 2 == 0) {
        const double elapsed = (this->now() - t_start).seconds();
        feedback->detections_so_far = detections;
        feedback->current_fps =
          elapsed > 0.0 ? static_cast<float>(detections / elapsed) : 0.0f;
        goal_handle->publish_feedback(feedback);
      }

      rate.sleep();
    }

    // Node shutting down (Ctrl+C): leave the goal in a terminal state.
    if (goal_handle->is_active()) {
      result->total_detections = detections;
      result->success = false;
      goal_handle->abort(result);
    }
    finishGoal(target, "node shutdown");
  }

  /// Release pipeline ownership and wake the next queued goal, if any.
  void finishGoal(const std::string & target, const char * reason)
  {
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      active_goal_.reset();
    }
    goal_done_cv_.notify_one();
    RCLCPP_INFO(this->get_logger(), "Detection for '%s' stopped (%s)",
                target.c_str(), reason);
  }

  /// Build one random detection honouring the CURRENT threshold.
  Detection makeDetection(const std::string & target_class)
  {
    const double threshold = this->get_parameter("confidence_threshold").as_double();

    std::uniform_real_distribution<float> conf_dist(static_cast<float>(threshold), 1.0f);
    std::uniform_real_distribution<float> x_dist(0.0f, 640.0f);
    std::uniform_real_distribution<float> y_dist(0.0f, 480.0f);

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

  // Members 

  rclcpp::Publisher<Detection>::SharedPtr detection_pub_;
  rclcpp::Service<SetConfidence>::SharedPtr set_confidence_srv_;
  rclcpp_action::Server<StartDetection>::SharedPtr action_server_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  std::mutex goal_mutex_;
  std::condition_variable goal_done_cv_;
  std::shared_ptr<GoalHandle> active_goal_;

  std::mt19937 rng_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionManager>());
  rclcpp::shutdown();
  return 0;
}