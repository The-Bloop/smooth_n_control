#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <iostream>
#include <fstream>
#include <sstream>

#include "smooth_n_control/srv/smooth_path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "PathMarkerPublisher.cpp"
#include "PathSegment.cpp"
#include "PathSampler.cpp"
#include "NewController.cpp"

using namespace std::chrono_literals;

// Helper: Convert a 2D vector to Float64MultiArray
smooth_n_control::msg::Path vector2DToPath(const std::vector<std::vector<double>> & vec) {
    smooth_n_control::msg::Path msg;
    if (vec.empty()) return msg;
    size_t rows = vec.size();
    for (size_t i = 0; i < rows; i++)
    {
        smooth_n_control::msg::Point2d point;
        point.x = vec[i][0];
        point.y = vec[i][1];
        msg.points.push_back(point);
    }

    return msg;
}

// Helper: Convert Float64MultiArray to 2D vector
std::vector<std::vector<double>> pathTo2DVector(const smooth_n_control::msg::Path & msg) {
    std::vector<std::vector<double>> result;
    if (msg.points.size() <= 2) return result;
    size_t rows = msg.points.size();
    size_t cols = 2;
    result.resize(rows, std::vector<double>(cols));
    for (size_t i = 0; i < rows; ++i)
    {
        result[i][0] = msg.points[i].x;
        result[i][1] = msg.points[i].y;
    }
    return result;
}



class SmoothPathClient : public rclcpp::Node
{
public:
    SmoothPathClient()
    : Node("smooth_path_client")
    {
        client_ = this->create_client<smooth_n_control::srv::SmoothPath>("smooth_path");
    }

    void create_marker_pubs()
    {
        smooth_marker_pub_ = std::make_shared<PathMarkerPublisher>(shared_from_this(), "smooth_path_plan");
        original_marker_pub = std::make_shared<PathMarkerPublisher>(shared_from_this(), "original_path_plan");
    }

    smooth_n_control::msg::Path send_and_show_path(smooth_n_control::msg::Path &path)
    {
        smooth_n_control::msg::Path smoothed_path;
        if (!client_->wait_for_service(10s)) {
            RCLCPP_ERROR(this->get_logger(), "Service not available after waiting. Quitting.");
            rclcpp::shutdown();
            return smoothed_path;
        }

        std::array<float, 4> original_color = {1.0, 0.0, 0.0, 1.0}; // Red RGBA
        original_marker_pub->publishPathMarker(path, original_color, 0.05);

        auto request = std::make_shared<smooth_n_control::srv::SmoothPath::Request>();
        request->path = path;

        RCLCPP_INFO(this->get_logger(), "Requesting path smoothing service...");
        auto future = client_->async_send_request(request);


        // Wait for the response (spin until ready)
        if (rclcpp::spin_until_future_complete(shared_from_this(), future) == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            smoothed_path = response->smoothed_path;

            std::array<float, 4> smooth_color = {0.0, 1.0, 0.0, 1.0}; // Green RGBA
            smooth_marker_pub_->publishPathMarker(smoothed_path, smooth_color, 0.05);

            RCLCPP_INFO(this->get_logger(), "Smoothed path published to RViz marker topic.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call smoothing service.");
        }

        return smoothed_path;
    }

private:
    rclcpp::Client<smooth_n_control::srv::SmoothPath>::SharedPtr client_;
    std::shared_ptr<PathMarkerPublisher> smooth_marker_pub_;
    std::shared_ptr<PathMarkerPublisher> original_marker_pub;
};

smooth_n_control::msg::Path read2DPointsFromFile(const std::string& file_path)
{
    smooth_n_control::msg::Path points;
    std::ifstream infile(file_path);
    if (!infile) {
        std::cerr << "Could not open file: " << file_path << std::endl;
        return points;
    }
    std::string line;
    while (std::getline(infile, line)) {
        if (line.empty()) continue;
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream iss(line);
        double x, y;
        if (!(iss >> x >> y)) {
            std::cerr << "Skipping invalid line: " << line << std::endl;
            continue;
        }
        smooth_n_control::msg::Point2d point;
        point.x = x;
        point.y = y;
        points.points.push_back(point);
    }
    return points;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto smooth_client_node = std::make_shared<SmoothPathClient>();
    smooth_client_node->create_marker_pubs();

    auto path_pub = smooth_client_node->create_publisher<smooth_n_control::msg::Path>("smoothed_path", 10);

    std::string file_path = argv[1];

    // Create Path
    auto path = read2DPointsFromFile(file_path);

    // Path Smoothing
    auto smoothed_path = smooth_client_node->send_and_show_path(path);

    path_pub->publish(smoothed_path);

    rclcpp::spin_some(smooth_client_node);
    
    // // //Trajectory Control
    // auto control_node = std::make_shared<PIDTrajectoryFollower>(all_trajectory);
    // RCLCPP_INFO(control_node->get_logger(), "Starting up the controller.");
    // rclcpp::spin(control_node);

    // rclcpp::shutdown();
    return 0;
}