#include "../include/smooth_n_control/MoveBotServer.hpp"

#include "smooth_n_control/msg/pose2d.hpp"
#include "smooth_n_control/msg/point2d.hpp"

MoveBotServer::MoveBotServer() : Node("move_bot_server")
{
  	using namespace std::placeholders;

  	this->server_ = rclcpp_action::create_server<MB>(
    	this,
    	"move_bot_controller",
    	std::bind(&MoveBotServer::handle_goal, this, _1, _2),
    	std::bind(&MoveBotServer::handle_cancel, this, _1),
    	std::bind(&MoveBotServer::handle_accepted, this, _1));

  	cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  	odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      	"/odom", 10, std::bind(&MoveBotServer::odom_callback, this, _1));

}

void MoveBotServer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  	robot_x_ = msg->pose.pose.position.x;
  	robot_y_ = msg->pose.pose.position.y;
  	// Convert quaternion to yaw
  	double qw = msg->pose.pose.orientation.w;
  	double qx = msg->pose.pose.orientation.x;
  	double qy = msg->pose.pose.orientation.y;
  	double qz = msg->pose.pose.orientation.z;
  	robot_yaw_ = std::atan2(2.0 * (qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz));
  	odom_received_ = true;
}

rclcpp_action::GoalResponse MoveBotServer::handle_goal(const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const MB::Goal> goal)
{
  	RCLCPP_INFO(this->get_logger(), "Received goal request with order");
    trajectory_ = goal->trajectory;
    move_flag = true;
  	(void)uuid;
  	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MoveBotServer::handle_cancel(const std::shared_ptr<GoalHandleMB> goal_handle)
{
  	RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  	(void)goal_handle;
  	return rclcpp_action::CancelResponse::ACCEPT;
}

void MoveBotServer::handle_accepted(const std::shared_ptr<GoalHandleMB> goal_handle)
{
  	using namespace std::placeholders;    
  	// this needs to return quickly to avoid blocking the executor, so spin up a new thread
  	std::thread{std::bind(&MoveBotServer::execute, this, _1), goal_handle}.detach();
}

void MoveBotServer::execute(const std::shared_ptr<GoalHandleMB> goal_handle)
{
	RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(2);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MB::Feedback>();
    auto result = std::make_shared<MB::Result>();

    while(move_flag)
    {
       this-> control_loop();
       smooth_n_control::msg::Pose2d current_pose;
       current_pose.point.x = robot_x_;
       current_pose.point.y = robot_y_;
       current_pose.yaw = robot_yaw_;

       feedback->current_pose = current_pose;
       feedback->next_pose = trajectory_.poses[goal_idx_];

       goal_handle->publish_feedback(feedback);

       loop_rate.sleep();
    }

    if (rclcpp::ok() and !move_flag) {
      result->message = "Goal Reached";
      goal_handle->succeed(result);
    }

}

int MoveBotServer::findNextIdx() const
{
    if(goal_idx_ >= (int)trajectory_.poses.size() - 1)
        return goal_idx_;

    int min_idx = 0;
    double min_dist = 1e9;
    for (int i = goal_idx_; i < goal_idx_ + 3; ++i) {
        double dx = robot_x_ - trajectory_.poses[i].point.x;
        double dy = robot_y_ - trajectory_.poses[i].point.y;
        double d = std::hypot(dx, dy);
        if (d < min_dist) { min_dist = d; min_idx = i; }
    }
    if (min_idx + 1 < static_cast<int>(trajectory_.poses.size()))
        return min_idx + 1;
    else
        return min_idx;
}

void MoveBotServer::control_loop()
{
    if (!odom_received_ || trajectory_.poses.empty()) return;

    int goal_idx = findNextIdx();
    auto goal_x = trajectory_.poses[goal_idx].point.x;
    auto goal_y = trajectory_.poses[goal_idx].point.y;
    auto goal_theta = trajectory_.poses[goal_idx].yaw;

    double dx = goal_x - robot_x_;
    double dy = goal_y - robot_y_;
    double rho = std::hypot(dx, dy);
    double path_yaw = std::atan2(dy, dx);
    double theta = path_yaw - robot_yaw_;
    theta = std::atan2(std::sin(theta), std::cos(theta));
    double yaw_error = goal_theta - robot_yaw_;
    yaw_error = std::atan2(std::sin(yaw_error), std::cos(yaw_error));

    // Control gains
    double Kp_lin = 0.9, Kp_ang = 2.8;
    double linear = Kp_lin * rho;
    double angular = Kp_ang * theta;

    // When close to goal, reorient to final yaw
    if (goal_idx == (int)trajectory_.poses.size() - 1 && rho < 0.100) {
        linear = 0.0;
        angular = 2.5 * yaw_error;
        if (std::fabs(yaw_error) < 0.1) {
            publish_zero(); // Stop at the final pose
            move_flag = false;
            return;
        }
    } else if (rho < 0.100) {
        linear = 0.0;
        angular = 1.7 * yaw_error;
    }

        // Limits
    linear  = std::clamp(linear, -1.0, 1.0);
    angular = std::clamp(angular, -2.5, 2.5);

    geometry_msgs::msg::Twist twist;
    twist.linear.x  = linear;
    twist.angular.z = angular;
    cmd_vel_pub_->publish(twist);
    goal_idx_ = goal_idx ;
}

void MoveBotServer::publish_zero()
{
    geometry_msgs::msg::Twist twist;
    cmd_vel_pub_->publish(twist);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);


    auto move_bot_server_node = std::make_shared<MoveBotServer>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(move_bot_server_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}