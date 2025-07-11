#include "../include/smooth_n_control/PathSegment.hpp"
#include <algorithm>

double PathSegmenter::angleBetween(const std::vector<double>& p0,
                                   const std::vector<double>& p1,
                                   const std::vector<double>& p2)
{
    double v1x = p1[0] - p0[0], v1y = p1[1] - p0[1];
    double v2x = p2[0] - p1[0], v2y = p2[1] - p1[1];
    double dot = v1x * v2x + v1y * v2y;
    double len1 = std::hypot(v1x, v1y);
    double len2 = std::hypot(v2x, v2y);
    if (len1 == 0.0 || len2 == 0.0) return 0.0;
    double c = std::max(-1.0, std::min(1.0, dot / (len1 * len2)));
    return std::acos(c);
}

void PathSegmenter::segment(const std::vector<std::vector<double>>& path)
{
    sections_.clear();
    if (path.size() < 3) return;

    Section current;
    current.points.push_back(path[0]);

    double first_angle = angleBetween(path[0], path[1], path[2]);
    SectionType last_type = (first_angle < straight_angle_thresh_rad_) ? STRAIGHT : CURVE;
    current.type = last_type;

    for (size_t i = 1; i < path.size() - 1; ++i) {
        double angle = angleBetween(path[i-1], path[i], path[i+1]);
        SectionType this_type = (angle < straight_angle_thresh_rad_) ? STRAIGHT : CURVE;

        if (this_type == last_type) {
            current.points.push_back(path[i]);
        } else {
            current.points.push_back(path[i]);
            if (current.points.size() >= (size_t)min_points_per_section_) {
                sections_.push_back(current);
            }
            current.points.clear();
            current.points.push_back(path[i]);
            current.type = this_type;
            last_type = this_type;
        }
    }

    // Always include the last path point
    current.points.push_back(path.back());
    if (current.points.size() >= (size_t)min_points_per_section_ || sections_.empty())
        sections_.push_back(current);
}