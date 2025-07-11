#ifndef POSE_ARROW_MARKER_PUBLISHER_H
#define POSE_ARROW_MARKER_PUBLISHER_H

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tuple>
#include <string>
#include <array>
#include <vector>


class TrajectMarkerPublish {
public:
    TrajectMarkerPublish(
        rclcpp::Node::SharedPtr node,
        const std::string& topic,
        double scale,
        const std::array<float,4>& color_rgba,
        const std::string& ns) ;

    void add_markers(const std::vector<std::tuple<double, double, double>>& poses);

    void publish();

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    double scale_;
    std::array<float,4> color_rgba_;
    std::string ns_;
    visualization_msgs::msg::MarkerArray marker_array_;
    int id_;
    geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) const;

};

#endif