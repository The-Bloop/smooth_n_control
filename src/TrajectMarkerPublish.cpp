#include "../include/smooth_n_control/TrajectMarkerPublish.hpp"



// #include "smooth_n_control/msg/pose2d.hpp"
// #include "smooth_n_control/msg/trajectory.hpp"


TrajectMarkerPublish::TrajectMarkerPublish(rclcpp::Node::SharedPtr node, const std::string& topic, double scale, const std::array<float,4>& color_rgba, const std::string& ns)
        : node_(node),
          scale_(scale),
          color_rgba_(color_rgba),
          ns_(ns)
{
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(topic, 10);
    id_ = 0;
}

void TrajectMarkerPublish::add_markers(const smooth_n_control::msg::Trajectory& trajectory)
{
    rclcpp::Time now = node_->now();
    for (const auto& pose : trajectory.poses) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = now;
        marker.ns = ns_;
        marker.id = id_++;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = pose.point.x;
        marker.pose.position.y = pose.point.y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation = yawToQuaternion(pose.yaw);
        marker.scale.x = scale_;         // Arrow shaft length
        marker.scale.y = scale_ * 0.3;   // Arrow shaft diameter
        marker.scale.z = scale_ * 0.3;   // Arrow head diameter
        marker.color.r = color_rgba_[0];
        marker.color.g = color_rgba_[1];
        marker.color.b = color_rgba_[2];
        marker.color.a = color_rgba_[3];
        marker.lifetime = rclcpp::Duration::from_seconds(0.0);
        marker_array_.markers.push_back(marker);
    }
}

void TrajectMarkerPublish::publish()
{
    marker_pub_->publish(marker_array_);
}

geometry_msgs::msg::Quaternion TrajectMarkerPublish::yawToQuaternion(double yaw) const
    {
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        geometry_msgs::msg::Quaternion quat_msg;
        tf2::convert(q, quat_msg);
        return quat_msg;
    }