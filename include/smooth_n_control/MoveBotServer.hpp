#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "smooth_n_control/msg/trajectory.hpp"
#include "smooth_n_control/msg/pose2d.hpp"
#include "smooth_n_control/action/move_bot.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

class MoveBotServer : public rclcpp::Node{
    public:
        using MB = smooth_n_control::action::MoveBot;
        using GoalHandleMB = rclcpp_action::ServerGoalHandle<MB>;
        MoveBotServer();

    private:
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void execute(const std::shared_ptr<GoalHandleMB> goal_handle);
        void handle_accepted(const std::shared_ptr<GoalHandleMB> goal_handle);
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const MB::Goal> goal);
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMB> goal_handle);

        // Controller Functions
        int findNextIdx() const;
        void control_loop();
        void publish_zero();
        
        rclcpp_action::Server<smooth_n_control::action::MoveBot>::SharedPtr server_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        

        int goal_idx_ = 0;
        double robot_x_ = 0.0, robot_y_ = 0.0, robot_yaw_ = 0.0;
        bool odom_received_ = false;
        smooth_n_control::msg::Trajectory trajectory_;
        bool move_flag = false;

};