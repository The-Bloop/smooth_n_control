#include "../include/smooth_n_control/SmoothPathServer.hpp"
#include "smooth_n_control/msg/path.hpp"
#include "smooth_n_control/msg/point2d.hpp"

#include "CSpline.cpp"

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

// Converts from 2D std::vector to ROS Float64MultiArray
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

SmoothPathServer::SmoothPathServer() : Node("smooth_path_server") {
    service_ = this->create_service<smooth_n_control::srv::SmoothPath>(
            "smooth_path",
            std::bind(&SmoothPathServer::handle_service, this,
                      std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "SmoothPath Server ready.");
}

void SmoothPathServer::handle_service(
        const std::shared_ptr<smooth_n_control::srv::SmoothPath::Request> request,
        std::shared_ptr<smooth_n_control::srv::SmoothPath::Response> response) {

    // Convert the input to std::vector<std::vector<double>>
        auto in_path = pathTo2DVector(request->path);

        CubicSpline2D spline(in_path);

        auto smoothed = spline.interpolate(10);

        // Convert back to ROS message
        response->smoothed_path = vector2DToPath(smoothed);
        RCLCPP_INFO(this->get_logger(), "Processed smooth_path service call.");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SmoothPathServer>());
    rclcpp::shutdown();
    
    return 0;
}