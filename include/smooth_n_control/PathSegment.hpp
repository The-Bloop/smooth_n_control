#ifndef PATH_SEGMENTER_H
#define PATH_SEGMENTER_H

#include <vector>
#include <string>
#include <cmath>

class PathSegmenter {
public:
    enum SectionType { STRAIGHT, CURVE };

    struct Section {
        SectionType type;
        std::vector<std::vector<double>> points; // always contains start, end, and all in between
    };

    PathSegmenter(double straight_angle_thresh_rad = 0.05, // around 3 deg
                  int min_points_per_section = 3)
        : straight_angle_thresh_rad_(straight_angle_thresh_rad),
          min_points_per_section_(min_points_per_section) {}

    // Segments the path and updates internal sections
    void segment(const std::vector<std::vector<double>>& path);

    // Get the segments
    const std::vector<Section>& getSections() const { return sections_; }

    // Utility
    static std::string typeToString(SectionType type) {
        return type == STRAIGHT ? "STRAIGHT" : "CURVE";
    }

private:
    double straight_angle_thresh_rad_;
    int min_points_per_section_;
    std::vector<Section> sections_;

    static double angleBetween(const std::vector<double>& p0,
                               const std::vector<double>& p1,
                               const std::vector<double>& p2);
};

#endif