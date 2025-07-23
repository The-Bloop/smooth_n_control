#ifndef POSE_ARROW_MARKER_PUBLISHER_H
#define POSE_ARROW_MARKER_PUBLISHER_H

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <string>
#include <vector>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include "smooth_n_control/msg/pose2d.hpp"
#include "smooth_n_control/msg/trajectory.hpp"


class TrajectMarkerPublish {
public:
    TrajectMarkerPublish(
        rclcpp::Node::SharedPtr node,
        const std::string& topic,
        double scale,
        const std::array<float,4>& color_rgba,
        const std::string& ns) ;

    void add_markers(const smooth_n_control::msg::Trajectory& trajectory);

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