#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <vector>
#include <array>
#include <string>

class PathMarkerPublisher
{
    public:
        PathMarkerPublisher(const rclcpp::Node::SharedPtr &node, const std::string &topic_name);

        void publishPathMarker(const std::vector<std::vector<double>>& path, const std::array<float, 4>& color, double line_width);
        
    private:
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};
