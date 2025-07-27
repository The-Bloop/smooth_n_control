#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "smooth_n_control/action/move_bot.hpp"
#include "smooth_n_control/msg/trajectory.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class MoveBotClient : public rclcpp::Node
{
public:
        using MB = smooth_n_control::action::MoveBot;
        using GoalHandleMB = rclcpp_action::ClientGoalHandle<MB>;

  explicit MoveBotClient()
  : Node("move_bot_client")
  {
    using namespace std::placeholders;

    this->client_ptr_ = rclcpp_action::create_client<MB>(
      this,
      "/move_bot_controller");
    
    trajectory_sub = create_subscription<smooth_n_control::msg::Trajectory>("/trajectory", 
        10, std::bind(&MoveBotClient::trajectory_callback, this, _1));
  }

private:
  rclcpp_action::Client<MB>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<smooth_n_control::msg::Trajectory>::SharedPtr trajectory_sub;

  void trajectory_callback(const smooth_n_control::msg::Trajectory::SharedPtr msg)
  {
    using namespace std::placeholders;

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = MB::Goal();
    goal_msg.trajectory = *msg.get();

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<MB>::SendGoalOptions();
    send_goal_options.result_callback =
      std::bind(&MoveBotClient::result_callback, this, _1);
    send_goal_options.goal_response_callback =
      std::bind(&MoveBotClient::mb_goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&MoveBotClient::feedback_callback, this, _1, _2);
    client_ptr_->async_send_goal(goal_msg, send_goal_options);

  }

  void mb_goal_response_callback(const GoalHandleMB::SharedPtr future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleMB::SharedPtr,
    const std::shared_ptr<const MB::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Current Pose (x,y,yaw): ";
    ss << "(" << feedback->current_pose.point.x << ", "<< feedback->current_pose.point.y << ", " << feedback->current_pose.yaw << ")" << "; ";
    ss << "Next Pose (x,y,yaw): ";
    ss << "(" << feedback->next_pose.point.x << ", "<< feedback->next_pose.point.y << ", " << feedback->next_pose.yaw << ")";
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback(const GoalHandleMB::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    std::stringstream ss;
    for (auto letter : result.result->message) {
      ss << (char)toupper(letter) << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }
}; 

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto move_bot_client_node = std::make_shared<MoveBotClient>();
    rclcpp::spin(move_bot_client_node);
    return 0;
}