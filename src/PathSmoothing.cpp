#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <iostream>
#include <fstream>
#include <sstream>

#include <std_msgs/msg/float64_multi_array.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include "smooth_n_control/srv/smooth_path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "PathMarkerPublisher.cpp"
#include "PathSegment.cpp"
#include "PathSampler.cpp"
#include "TrajectMarkerPublish.cpp"
#include "NewController.cpp"

using namespace std::chrono_literals;

// Helper: Convert a 2D vector to Float64MultiArray
std_msgs::msg::Float64MultiArray vector2DToMultiArray(const std::vector<std::vector<double>> & vec) {
    std_msgs::msg::Float64MultiArray msg;
    if (vec.empty()) return msg;
    size_t rows = vec.size();
    size_t cols = vec[0].size();
    msg.layout.dim.resize(2);
    msg.layout.dim[0].label = "rows";
    msg.layout.dim[0].size = rows;
    msg.layout.dim[0].stride = rows * cols;
    msg.layout.dim[1].label = "cols";
    msg.layout.dim[1].size = cols;
    msg.layout.dim[1].stride = cols;
    msg.layout.data_offset = 0;
    msg.data.resize(rows * cols);
    for (size_t i = 0; i < rows; ++i)
        for (size_t j = 0; j < cols; ++j)
            msg.data[i * cols + j] = vec[i][j];
    return msg;
}

// Helper: Convert Float64MultiArray to 2D vector
std::vector<std::vector<double>> multiArrayTo2DVector(const std_msgs::msg::Float64MultiArray & msg) {
    std::vector<std::vector<double>> result;
    if (msg.layout.dim.size() != 2) return result;
    size_t rows = msg.layout.dim[0].size;
    size_t cols = msg.layout.dim[1].size;
    result.resize(rows, std::vector<double>(cols));
    for (size_t i = 0; i < rows; ++i)
        for (size_t j = 0; j < cols; ++j)
            result[i][j] = msg.data[i * cols + j];
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

    std::vector<std::vector<double>> send_and_show_path(const std::vector<std::vector<double>> &path)
    {
        std::vector<std::vector<double>> smoothed_path;
        if (!client_->wait_for_service(10s)) {
            RCLCPP_ERROR(this->get_logger(), "Service not available after waiting. Quitting.");
            rclcpp::shutdown();
            return smoothed_path;
        }

        std::array<float, 4> original_color = {1.0, 0.0, 0.0, 1.0}; // Red RGBA
        original_marker_pub->publishPathMarker(path, original_color, 0.05);

        auto request = std::make_shared<smooth_n_control::srv::SmoothPath::Request>();
        request->path = vector2DToMultiArray(path);

        RCLCPP_INFO(this->get_logger(), "Requesting path smoothing service...");
        auto future = client_->async_send_request(request);


        // Wait for the response (spin until ready)
        if (rclcpp::spin_until_future_complete(shared_from_this(), future) == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            smoothed_path = multiArrayTo2DVector(response->smoothed_path);

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

std::vector<std::vector<double>> read2DPointsFromFile(const std::string& file_path)
{
    std::vector<std::vector<double>> points;
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
        points.push_back({x, y});
    }
    return points;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto client_node = std::make_shared<SmoothPathClient>();
    client_node->create_marker_pubs();

    std::string file_path = argv[1];

    // Create Path
    auto path = read2DPointsFromFile(file_path);

    // Path Smoothing
    auto smoothed_path = client_node->send_and_show_path(path);

    rclcpp::spin_some(client_node);

    // Path Segmentation
    PathSegmenter segmenter(0.05, 3);
    segmenter.segment(smoothed_path);

    const auto& sections = segmenter.getSections();

    // Path Sampling for Trajectory points
    double v_max = 1.0;   // velocity
    double a_max = 0.5;   // acceleration
    double dt = 0.25;      // time step

    PathSampler sampler(v_max, a_max, dt);

    double v_start = 0.0, v_end = 0.0;
    double v_prev = v_start;

    auto node = rclcpp::Node::make_shared("arrow_marker_publisher_node");

    std::array<float, 4> traj_color = {1.0, 0.0, 1.0, 1.0};

    TrajectMarkerPublish marker_pub(node, "pose_arrows", 0.1, traj_color, "a");

    std::vector<std::tuple<double, double, double>> all_poses;

    //Trajectory Generation and Marker Creation
    for(size_t i = 0; i < sections.size(); ++i)
    {
        double curr_v_start = v_prev;
        double curr_v_end = (i < sections.size()-1) ? v_max : v_end;
        auto poses = sampler.sampleSection(sections[i].points, curr_v_start, curr_v_end);
        marker_pub.add_markers(poses);
        all_poses.insert(all_poses.end(), poses.begin(), poses.end());
        v_prev = v_max;
    }

    marker_pub.publish();

    rclcpp::spin_some(node);

    //Trajectory Control
    auto control_node = std::make_shared<PIDTrajectoryFollower>(all_poses);
    rclcpp::spin(control_node);

    rclcpp::shutdown();
    return 0;
}