#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "smooth_n_control/srv/smooth_path.hpp" // Replace with your actual package name
#include "smooth_n_control/msg/path.hpp"

#include "CSpline.hpp"

class SmoothPathServer : public rclcpp::Node{
    public: 
        SmoothPathServer();
    
    private:
        void handle_service(const std::shared_ptr<smooth_n_control::srv::SmoothPath::Request> request,
        std::shared_ptr<smooth_n_control::srv::SmoothPath::Response> response);

        rclcpp::Service<smooth_n_control::srv::SmoothPath>::SharedPtr service_;
};