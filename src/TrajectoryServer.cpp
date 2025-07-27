#include "../include/smooth_n_control/TrajectoryServer.hpp"

#include "PathSampler.cpp"
#include "PathSegment.cpp"

TrajectoryServer::TrajectoryServer() : Node("generate_trajectory_server") {
    service_ = this->create_service<smooth_n_control::srv::CreateTrajectory>(
            "generate_trajectory",
            std::bind(&TrajectoryServer::handle_service, this,
                      std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Trajectory Server ready.");
}

void TrajectoryServer::handle_service(const std::shared_ptr<smooth_n_control::srv::CreateTrajectory::Request> request,
        std::shared_ptr<smooth_n_control::srv::CreateTrajectory::Response> response){
    
    auto smoothed_path = request->smoothed_path;

    PathSegmenter segmenter(0.05, 3);
    segmenter.segment(smoothed_path);

    const auto& sections = segmenter.getSections();

    // Path Sampling for Trajectory points
    double v_max = 1.0;   // velocity
    double a_max = 0.5;   // acceleration
    double dt = 0.15;      // time step

    PathSampler sampler(v_max, a_max, dt);

    double v_start = 0.0, v_end = 0.0;
    double v_prev = v_start;

    smooth_n_control::msg::Trajectory all_trajectory;

    for(size_t i = 0; i < sections.size(); ++i)
    {
        double curr_v_start = v_prev;
        double curr_v_end = (i < sections.size()-1) ? v_max : v_end;
        auto trajectory = sampler.sampleSection(sections[i].points, curr_v_start, curr_v_end);
        all_trajectory.poses.insert(all_trajectory.poses.end(), trajectory.poses.begin(), trajectory.poses.end());
        v_prev = v_max;
    }
    response->trajectory = all_trajectory;
    RCLCPP_INFO(this->get_logger(), "Trajectories generated and returned.");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryServer>());
    rclcpp::shutdown();
    
    return 0;
}
