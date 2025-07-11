#include "../include/smooth_n_control/SmoothPathServer.hpp"

#include "CSpline.cpp"

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

// Converts from 2D std::vector to ROS Float64MultiArray
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
        auto in_path = multiArrayTo2DVector(request->path);

        // Smooth it
        // RDP rdp_algo(1.0);          // You can parametrise these
        // Chaikin chaikin_algo(3);

        CubicSpline2D spline(in_path);

        // auto simplified = rdp_algo.simplify(in_path);
        // auto smoothed = chaikin_algo.smooth(in_path);

        auto smoothed = spline.interpolate(10);

        // Convert back to ROS message
        response->smoothed_path = vector2DToMultiArray(smoothed);
        RCLCPP_INFO(this->get_logger(), "Processed smooth_path service call.");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SmoothPathServer>());
    rclcpp::shutdown();
    
    return 0;
}