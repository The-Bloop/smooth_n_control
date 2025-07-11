#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdlib>

#include "nav_msgs/msg/occupancy_grid.hpp"


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("empty_map_publisher");

    auto pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

    // Create empty OccupancyGrid (size 0x0)
    nav_msgs::msg::OccupancyGrid map_msg;
    map_msg.header.frame_id = "map";
    // map_msg.header.stamp = node->now();
    map_msg.info.resolution = 1.0;         
    map_msg.info.width = 00;                 
    map_msg.info.height = 00;
    map_msg.info.origin.position.x = 0.0;
    map_msg.info.origin.position.y = 0.0;
    map_msg.info.origin.position.z = 0.0;
    map_msg.info.origin.orientation.w = 1.0;
    // No data (or: map_msg.data can be a zero-length array)

    // Publish ONCE
    
    while(rclcpp::ok())
    {
        map_msg.header.stamp = node->now();
        pub->publish(map_msg);
        rclcpp::sleep_for(std::chrono::milliseconds(500));
    }
    RCLCPP_INFO(node->get_logger(), "Published empty map to /map");
    // rclcpp::sleep_for(std::chrono::milliseconds(500));

    rclcpp::shutdown();
    return 0;
}