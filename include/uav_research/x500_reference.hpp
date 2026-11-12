#pragma once
#include "uav_research/p01_reference.hpp"
#include <algorithm>
#include <array>
#include <stdexcept>

namespace uav_research
{
// Integral of a seventh-degree smoothstep. Value and first four derivatives
// match the stationary/constant-speed segments at both ends of the ramp.
inline std::array<double, 5> phase_ramp(double t, double ramp)
{
    if (t <= 0) return {{0, 0, 0, 0, 0}};
    if (t >= ramp) return {{t-ramp/2, 1, 0, 0, 0}};
    // Evaluate near the upper endpoint through symmetry to avoid cancellation
    // in the high-order derivatives (important for snap/feedforward).
    if (t > ramp/2)
    {
        const auto reflected = phase_ramp(ramp-t, ramp);
        return {{t-ramp/2+reflected[0], 1-reflected[1], reflected[2],
            -reflected[3], reflected[4]}};
    }
    const double u = t/ramp;
    const auto pow = [u](int n) { return std::pow(u, n); };
    return {{ramp*(7*pow(5)-14*pow(6)+10*pow(7)-2.5*pow(8)),
        35*pow(4)-84*pow(5)+70*pow(6)-20*pow(7),
        (140*pow(3)-420*pow(4)+420*pow(5)-140*pow(6))/ramp,
        (420*pow(2)-1680*pow(3)+2100*pow(4)-840*pow(5))/(ramp*ramp),
        (840*u-5040*pow(2)+8400*pow(3)-4200*pow(4))/(ramp*ramp*ramp)}};
}

// Modified Case I for Gazebo: translate to the airborne start, rotate by its
// heading, slow down, smoothly start/stop. Not the paper's original experiment.
inline P01Reference reference_x500_case1(double t, const Eigen::Vector3d &origin,
    const Eigen::Vector3d &heading, double speed = 0.25, double ramp = 4.0,
    double duration = 30.0, bool rotate_heading = false)
{
    if (!std::isfinite(t) || !std::isfinite(speed) || speed <= 0 ||
        !std::isfinite(ramp) || ramp <= 0 || !std::isfinite(duration) || duration < 2*ramp)
        throw std::invalid_argument("Invalid X500 trajectory timing");
    auto d = phase_ramp(t, ramp);
    const auto stop = phase_ramp(t-(duration-ramp), ramp);
    for (int i = 0; i < 5; ++i) d[i] = speed*(d[i]-stop[i]);
    const auto q = reference_case1(d[0]);
    const auto hold = reference_hover(origin, heading);
    Eigen::Matrix3d rotation;
    rotation.col(0) = hold.b1;
    rotation.col(1) = Eigen::Vector3d::UnitZ().cross(hold.b1);
    rotation.col(2) = Eigen::Vector3d::UnitZ();
    P01Reference r;
    r.x = origin + rotation*(q.x-reference_case1(0).x);
    r.v = rotation*(q.v*d[1]);
    r.a = rotation*(q.a*d[1]*d[1]+q.v*d[2]);
    r.j = rotation*(q.j*std::pow(d[1], 3)+3*q.a*d[1]*d[2]+q.v*d[3]);
    r.snap = rotation*(q.snap*std::pow(d[1], 4)+6*q.j*d[1]*d[1]*d[2]+
        3*q.a*d[2]*d[2]+4*q.a*d[1]*d[3]+q.v*d[4]);
    r.b1 = rotate_heading ? Eigen::Vector3d(rotation*q.b1) : hold.b1;
    r.b1_dot = rotate_heading ? Eigen::Vector3d(rotation*q.b1_dot*d[1]) : Eigen::Vector3d::Zero();
    r.b1_ddot = rotate_heading ? Eigen::Vector3d(rotation*(q.b1_ddot*d[1]*d[1]+q.b1_dot*d[2])) : Eigen::Vector3d::Zero();
    return r;
}
} // namespace uav_research
