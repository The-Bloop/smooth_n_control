#ifndef CUBIC_SPLINE_2D_H
#define CUBIC_SPLINE_2D_H

#include <vector>
#include <cstddef>

// CubicSpline2D: fit a smooth parametric cubic spline through 2D path
class CubicSpline2D {
public:
    // Construct and fit the spline
    CubicSpline2D(const std::vector<std::vector<double>>& path)
    {
        fit(path);
    }

    // Get interpolated points. Output contains ALL original points (if sample_per_segment >= 1)
    std::vector<std::vector<double>> interpolate(int sample_per_segment = 10) const;

private:
    std::vector<double> t, x, y;
    std::vector<double> ax, bx, cx, dx; // Spline coefs for x
    std::vector<double> ay, by, cy, dy; // Spline coefs for y

    // Fit the spline (parametric: x(t), y(t)), t is cumulative arc-length
    void fit(const std::vector<std::vector<double>>& path);

    void compute_coeffs(const std::vector<double>& t, const std::vector<double>& v,
                        std::vector<double>& a, std::vector<double>& b, std::vector<double>& c, std::vector<double>& d) const;

    double eval(const std::vector<double>& a, const std::vector<double>& b,
                const std::vector<double>& c, const std::vector<double>& d,
                int idx, double dt) const;
};

#endif