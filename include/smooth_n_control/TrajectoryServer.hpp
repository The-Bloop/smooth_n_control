#include "rclcpp/rclcpp.hpp"
#include "smooth_n_control/srv/create_trajectory.hpp" // Replace with your actual package name
#include "smooth_n_control/msg/path.hpp"
#include "smooth_n_control/msg/trajectory.hpp"

#include "PathSampler.hpp"
#include "PathSegment.hpp"
#include "TrajectMarkerPublish.hpp"

class TrajectoryServer : public rclcpp::Node{
    public:
        TrajectoryServer();

    private:
        void handle_service(const std::shared_ptr<smooth_n_control::srv::CreateTrajectory::Request> request,
        std::shared_ptr<smooth_n_control::srv::CreateTrajectory::Response> response);

        rclcpp::Service<smooth_n_control::srv::CreateTrajectory>::SharedPtr service_;

};


