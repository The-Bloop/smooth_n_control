#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <vector>
#include <tuple>
#include <cmath>
#include <algorithm>


class PIDTrajectoryFollower : public rclcpp::Node
{
public:
    PIDTrajectoryFollower(const std::vector<std::tuple<double, double, double>>& trajectory)
    : Node("pid_trajectory_follower"), trajectory_(trajectory)
    {
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&PIDTrajectoryFollower::odom_callback, this, std::placeholders::_1));
        timer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&PIDTrajectoryFollower::control_loop, this));
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
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

    int findNextIdx() const
    {
        if(goal_idx_ >= (int)trajectory_.size() - 1)
            return goal_idx_;

        int min_idx = 0;
        double min_dist = 1e9;
        for (int i = goal_idx_; i < goal_idx_ + 3; ++i) {
            double dx = robot_x_ - std::get<0>(trajectory_[i]);
            double dy = robot_y_ - std::get<1>(trajectory_[i]);
            double d = std::hypot(dx, dy);
            if (d < min_dist) { min_dist = d; min_idx = i; }
        }
        if (min_idx + 1 < static_cast<int>(trajectory_.size()))
            return min_idx + 1;
        else
            return min_idx;
    }

    void control_loop()
    {
        if (!odom_received_ || trajectory_.empty()) return;

        int goal_idx = findNextIdx();
        auto [goal_x, goal_y, goal_theta] = trajectory_.at(goal_idx);

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
        if (goal_idx == (int)trajectory_.size() - 1 && rho < 0.250) {
            linear = 0.0;
            angular = 2.5 * yaw_error;
            std::cout<<"Yaw Error: "<< std::fabs(yaw_error)<< "\n";
            if (std::fabs(yaw_error) < 0.1) {
                publish_zero(); // Stop at the final pose
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Goal reached, stopped.");
                rclcpp::shutdown();
                return;
            }
        } else if (rho < 0.250) {
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

    void publish_zero()
    {
        geometry_msgs::msg::Twist twist;
        cmd_vel_pub_->publish(twist);
    }

    std::vector<std::tuple<double, double, double>> trajectory_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    int goal_idx_ = 0;

    double robot_x_ = 0.0, robot_y_ = 0.0, robot_yaw_ = 0.0;
    bool odom_received_ = false;
};