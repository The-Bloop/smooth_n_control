#include "../include/smooth_n_control/PathSegment.hpp"
#include "smooth_n_control/msg/path.hpp"
#include "smooth_n_control/msg/point2d.hpp"
#include <algorithm>

double PathSegmenter::angleBetween(const smooth_n_control::msg::Point2d& p0,
                                   const smooth_n_control::msg::Point2d& p1,
                                   const smooth_n_control::msg::Point2d& p2)
{
    double v1x = p1.x - p0.x, v1y = p1.y - p0.y;
    double v2x = p2.x - p1.x, v2y = p2.y - p1.y;
    double dot = v1x * v2x + v1y * v2y;
    double len1 = std::hypot(v1x, v1y);
    double len2 = std::hypot(v2x, v2y);
    if (len1 == 0.0 || len2 == 0.0) return 0.0;
    double c = std::max(-1.0, std::min(1.0, dot / (len1 * len2)));
    return std::acos(c);
}

void PathSegmenter::segment(const smooth_n_control::msg::Path& path)
{
    sections_.clear();
    if (path.points.size() < 3) return;

    Section current;
    current.points.push_back({path.points[0].x,path.points[0].y});

    double first_angle = angleBetween(path.points[0], path.points[1], path.points[2]);
    SectionType last_type = (first_angle < straight_angle_thresh_rad_) ? STRAIGHT : CURVE;
    current.type = last_type;

    for (size_t i = 1; i < path.points.size() - 1; ++i) {
        double angle = angleBetween(path.points[i-1], path.points[i], path.points[i+1]);
        SectionType this_type = (angle < straight_angle_thresh_rad_) ? STRAIGHT : CURVE;

        if (this_type == last_type) {
            current.points.push_back({path.points[i].x,path.points[i].y});
        } else {
            current.points.push_back({path.points[i].x,path.points[i].y});
            if (current.points.size() >= (size_t)min_points_per_section_) {
                sections_.push_back(current);
            }
            current.points.clear();
            current.points.push_back({path.points[i].x,path.points[i].y});
            current.type = this_type;
            last_type = this_type;
        }
    }

    // Always include the last path point
    current.points.push_back({path.points.back().x,path.points.back().y});
    if (current.points.size() >= (size_t)min_points_per_section_ || sections_.empty())
        sections_.push_back(current);
}