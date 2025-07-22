#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <vector>
#include <array>
#include <string>

class PathMarkerPublisher
{
    public:
        PathMarkerPublisher(const rclcpp::Node::SharedPtr &node, const std::string &topic_name);

        void publishPathMarker(const smooth_n_control::msg::Path& path, const std::array<float, 4>& color, double line_width);
        
    private:
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};
