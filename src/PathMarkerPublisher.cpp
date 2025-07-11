#include "../include/smooth_n_control/PathMarkerPublisher.hpp"

PathMarkerPublisher::PathMarkerPublisher(const rclcpp::Node::SharedPtr &node, const std::string &topic_name){
    marker_pub_ = node->create_publisher<visualization_msgs::msg::Marker>(topic_name, 10);
}

void PathMarkerPublisher::publishPathMarker(const std::vector<std::vector<double>>& path, const std::array<float, 4>& color, double line_width)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "smoothed_path";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = line_width; // Thickness

    marker.pose.orientation.w = 1.0;

    // Set color
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = color[3];

    // Convert path points to geometry_msgs::msg::Point
    for (const auto& pt : path) {
        geometry_msgs::msg::Point p;
        p.x = pt[0];
        p.y = pt[1];
        p.z = 0.0;
        marker.points.push_back(p);
    }

    marker.lifetime = rclcpp::Duration::from_seconds(0.0); // stays until updated

    marker_pub_->publish(marker);
}
