#ifndef PATH_SECTION_SAMPLER_H
#define PATH_SECTION_SAMPLER_H

#include <vector>
#include <tuple>

#include "smooth_n_control/msg/pose2d.hpp"
#include "smooth_n_control/msg/trajectory.hpp"

class PathSampler {
public:
    PathSampler(double v_max, double a_max, double dt)
        : v_max_(v_max), a_max_(a_max), dt_(dt) {}

    // Main function: sample (x, y, theta) at dt intervals along a section's points,
    // starting at v_start and ending at v_end (m/s)
    smooth_n_control::msg::Trajectory
    sampleSection(const std::vector<std::vector<double>>& section_points,
                  double v_start = 0.0, double v_end = 0.0);

private:
    double v_max_;
    double a_max_;
    double dt_;

    struct TrapezoidProfile {
        double total_dist;
        double t_accel, t_cruise, t_decel, t_total;
        double d_accel, d_cruise, d_decel;
        double v_start, v_cruise, v_end;
    };

    std::vector<double> computeCumulativeDistance(
        const std::vector<std::vector<double>>& pts) const;

    TrapezoidProfile calcTrapezoidProfile(double total_dist, double v_start, double v_end) const;

    double trapDistanceAtTime(const TrapezoidProfile& prof, double t) const;

    smooth_n_control::msg::Pose2d
    interpolateSectionPose(const std::vector<std::vector<double>>& pts,
                          const std::vector<double>& cum_dist,
                          double d_query) const;
};

#endif