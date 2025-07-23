#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <iostream>
#include <fstream>
#include <sstream>

#include "smooth_n_control/srv/create_trajectory.hpp"
#include "smooth_n_control/msg/path.hpp"
#include "smooth_n_control/msg/point2d.hpp"

#include "TrajectMarkerPublish.cpp"

class TrajectoryClient : public rclcpp::Node
{
public:
    TrajectoryClient() 
    : Node("trajectory_client")
    {
        client_ = this->create_client<smooth_n_control::srv::CreateTrajectory>("generate_trajectory");
        path_sub_ = create_subscription<smooth_n_control::msg::Path>(
            "/smoothed_path", 10, std::bind(&TrajectoryClient::path_callback, this, std::placeholders::_1));
    }

    void create_pubs()
    {
        trajectory_pub_ = this->create_publisher<smooth_n_control::msg::Trajectory>("trajectory", 10);
        std::array<float, 4> traj_color = {1.0, 0.0, 1.0, 1.0};
        trajectory_marker_pub_ = std::make_shared<TrajectMarkerPublish>(shared_from_this(), "pose_arrows", 0.1, traj_color, "a");
    }

private:

    void path_callback(const smooth_n_control::msg::Path::SharedPtr msg)
    {
        if (!client_->wait_for_service(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(this->get_logger(), "Service not available after waiting. Quitting.");
            rclcpp::shutdown();
        }

        auto request = std::make_shared<smooth_n_control::srv::CreateTrajectory::Request>();
        request->smoothed_path = *msg.get();

        RCLCPP_INFO(this->get_logger(), "Requesting trajectory generation service...");
        auto future = client_->async_send_request(request,
            std::bind(&TrajectoryClient::service_response_callback, this, std::placeholders::_1));

    }

    void service_response_callback(rclcpp::Client<smooth_n_control::srv::CreateTrajectory>::SharedFuture future)
    {
        RCLCPP_INFO(this->get_logger(), "Trajectories successfully generated. Publishing trajectory and markers.");
        auto response = future.get();
        trajectory_marker_pub_->add_markers(response->trajectory);
        trajectory_marker_pub_->publish();
        trajectory_pub_->publish(response->trajectory);
    }

    rclcpp::Client<smooth_n_control::srv::CreateTrajectory>::SharedPtr client_;
    std::shared_ptr<TrajectMarkerPublish> trajectory_marker_pub_;
    rclcpp::Subscription<smooth_n_control::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<smooth_n_control::msg::Trajectory>::SharedPtr trajectory_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto traject_client_node = std::make_shared<TrajectoryClient>();
    traject_client_node->create_pubs();
    rclcpp::spin(traject_client_node);
    return 0;
}