#include "../include/smooth_n_control/PathSampler.hpp"
#include <cmath>
#include "smooth_n_control/msg/pose2d.hpp"
#include "smooth_n_control/msg/trajectory.hpp"

// ---------- Internal helpers ------------

std::vector<double> PathSampler::computeCumulativeDistance(
    const std::vector<std::vector<double>>& pts) const
{
    std::vector<double> cum_dist(pts.size(), 0.0);
    for (size_t i = 1; i < pts.size(); ++i) {
        double dx = pts[i][0] - pts[i-1][0];
        double dy = pts[i][1] - pts[i-1][1];
        cum_dist[i] = cum_dist[i-1] + std::sqrt(dx*dx + dy*dy);
    }
    return cum_dist;
}

// Calculates a general trapezoidal profile from v_start, v_end, subject to v_max and a_max
PathSampler::TrapezoidProfile
PathSampler::calcTrapezoidProfile(double total_dist, double v_start, double v_end) const
{
    TrapezoidProfile prof;
    prof.v_start = v_start;
    prof.v_end = v_end;

    // Max velocity allowed by user
    double v_cruise = v_max_;

    // Time and distance to accelerate from v_start to v_cruise
    double t_accel = (v_cruise - v_start) / a_max_;
    double d_accel = (v_start + v_cruise) * t_accel * 0.5;

    // Time and distance to decelerate from v_cruise to v_end
    double t_decel = (v_cruise - v_end) / a_max_;
    double d_decel = (v_end + v_cruise) * t_decel * 0.5;

    // Check if a cruise segment is possible; adjust for triangular profile
    double d_cruise = total_dist - d_accel - d_decel;
    if (d_cruise < 0) { 
        // No cruise, triangular profile. Recalculate v_peak.
        // v_peak^2 = 0.5*(v_start^2 + v_end^2) + a*total_dist
        v_cruise = std::sqrt(
            0.5 * (v_start * v_start + v_end * v_end) + a_max_ * total_dist);
        t_accel = std::max(0.0, (v_cruise - v_start) / a_max_);
        t_decel = std::max(0.0, (v_cruise - v_end) / a_max_);
        d_accel = (v_start + v_cruise) * t_accel * 0.5;
        d_decel = (v_end + v_cruise) * t_decel * 0.5;
        d_cruise = 0.0;
        prof.t_cruise = 0.0;
    }

    prof.v_cruise = v_cruise;
    prof.t_accel = t_accel;
    prof.t_decel = t_decel;
    prof.t_cruise = (d_cruise > 0) ? (d_cruise / v_cruise) : 0.0;
    prof.d_accel = d_accel;
    prof.d_cruise = d_cruise;
    prof.d_decel = d_decel;
    prof.t_total = prof.t_accel + prof.t_cruise + prof.t_decel;
    prof.total_dist = total_dist;
    return prof;
}

// Distance traversed along the profile at time t
double PathSampler::trapDistanceAtTime(const TrapezoidProfile& prof, double t) const
{
    if (t < 0) return 0.0;
    if (t <= prof.t_accel) {
        return prof.v_start * t + 0.5 * a_max_ * t * t;
    } else if (t <= prof.t_accel + prof.t_cruise) {
        double t_c = t - prof.t_accel;
        return prof.d_accel + prof.v_cruise * t_c;
    } else if (t <= prof.t_total) {
        double t_d = t - prof.t_accel - prof.t_cruise;
        return prof.d_accel + prof.d_cruise +
            prof.v_cruise * t_d - 0.5 * a_max_ * t_d * t_d;
    } else {
        return prof.total_dist;
    }
}

smooth_n_control::msg::Pose2d
PathSampler::interpolateSectionPose(const std::vector<std::vector<double>>& pts,
                                           const std::vector<double>& cum_dist,
                                           double d_query) const
{
    smooth_n_control::msg::Pose2d pose;
    size_t idx = 0;
    while (idx + 1 < cum_dist.size() && cum_dist[idx+1] < d_query) ++idx;
    if (idx+1 == cum_dist.size()) idx = cum_dist.size()-2;
    double seg_len = cum_dist[idx+1] - cum_dist[idx];
    double frac = (seg_len > 0) ? (d_query - cum_dist[idx]) / seg_len : 0.0;
    pose.point.x = pts[idx][0] + frac * (pts[idx+1][0] - pts[idx][0]);
    pose.point.y = pts[idx][1] + frac * (pts[idx+1][1] - pts[idx][1]);
    double dx = pts[idx+1][0] - pts[idx][0];
    double dy = pts[idx+1][1] - pts[idx][1];
    pose.yaw = std::atan2(dy, dx);
    return pose;
}

// ---------- Main API ------------

smooth_n_control::msg::Trajectory
PathSampler::sampleSection(const std::vector<std::vector<double>>& section_points,
                                 double v_start, double v_end)
{
    smooth_n_control::msg::Trajectory trajectory;
    if(section_points.size() < 2) return trajectory;

    auto cum_dist = computeCumulativeDistance(section_points);
    double total_dist = cum_dist.back();
    if (total_dist <= 0.0) return trajectory;
    TrapezoidProfile prof = calcTrapezoidProfile(total_dist, v_start, v_end);

    for(double t = 0; t < prof.t_total; t += dt_) {
        double d = trapDistanceAtTime(prof, t);
        trajectory.poses.push_back(interpolateSectionPose(section_points, cum_dist, d));
    }
    // Ensure endpoint is included
    // trajectory.poses.push_back(interpolateSectionPose(section_points, cum_dist, total_dist));
    return trajectory;
}