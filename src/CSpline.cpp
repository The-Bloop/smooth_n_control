#include "../include/smooth_n_control/CSpline.hpp"
#include <cmath>
#include <cassert>

// --------- Internals: tridiagonal system solver for cubic spline --------------
void CubicSpline2D::compute_coeffs(const std::vector<double>& t, const std::vector<double>& v,
                                   std::vector<double>& a, std::vector<double>& b,
                                   std::vector<double>& c, std::vector<double>& d) const
{
    int n = t.size();
    std::vector<double> h(n-1), alpha(n-1);
    for (int i=0;i<n-1;++i) h[i] = t[i+1] - t[i];
    for (int i=1;i<n-1;++i)
        alpha[i] = 3*( (v[i+1] - v[i]) / h[i] - (v[i] - v[i-1]) / h[i-1] );

    std::vector<double> l(n), mu(n), z(n);
    l[0]=1; mu[0]=z[0]=0;
    for (int i=1;i<n-1;++i) {
        l[i] = 2*(t[i+1]-t[i-1]) - h[i-1]*mu[i-1];
        mu[i] = h[i] / l[i];
        z[i] = (alpha[i] - h[i-1]*z[i-1]) / l[i];
    }
    l[n-1]=1; z[n-1]=0;

    c.assign(n,0.0); b.assign(n-1,0.0); d.assign(n-1,0.0);
    for (int j=n-2;j>=0;--j) {
        c[j] = z[j] - mu[j]*c[j+1];
        b[j] = (v[j+1] - v[j])/h[j] - h[j]*(c[j+1]+2*c[j])/3;
        d[j] = (c[j+1] - c[j])/ (3*h[j]);
    }
    a = v;
}

// --------- Spline object building ----------
void CubicSpline2D::fit(const std::vector<std::vector<double>>& path)
{
    int n = path.size();
    assert(n >= 2);
    // Parametrize by cumulative chord length
    t.resize(n); x.resize(n); y.resize(n);
    t[0]=0;
    for(int i=0;i<n;++i){
        x[i] = path[i][0];
        y[i] = path[i][1];
        if(i>0){
            double dx = x[i]-x[i-1], dy = y[i]-y[i-1];
            t[i] = t[i-1] + std::sqrt(dx*dx+dy*dy);
        }
    }
    compute_coeffs(t,x,ax,bx,cx,dx);
    compute_coeffs(t,y,ay,by,cy,dy);
}

// --------- Spline evaluation ---------------
double CubicSpline2D::eval(const std::vector<double>& a, const std::vector<double>& b,
                           const std::vector<double>& c, const std::vector<double>& d,
                           int idx, double dt) const
{
    return a[idx] + b[idx]*dt + c[idx]*dt*dt + d[idx]*dt*dt*dt;
}

// -------- Main API: get smooth path -----------
std::vector<std::vector<double>> CubicSpline2D::interpolate(int sample_per_segment) const
{
    std::vector<std::vector<double>> pts;
    int n = t.size();
    for(int i=0;i<n-1;++i){
        for(int j=0;j<sample_per_segment;++j){
            double ratio = double(j)/sample_per_segment;
            double ti = t[i] + (t[i+1]-t[i])*ratio;
            double dt = ti - t[i];
            double px = eval(ax,bx,cx,dx,i,dt);
            double py = eval(ay,by,cy,dy,i,dt);
            pts.push_back({px,py});
        }
    }
    // Include the final original point for exactness!
    pts.push_back({x.back(),y.back()});
    return pts;
}