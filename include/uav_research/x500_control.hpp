#pragma once

#include "uav_research/p01_geometric_controller.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace uav_research
{
// Engineering starting values for Gazebo X500, NOT parameters from the paper.
inline P01Parameters x500_parameters()
{
    P01Parameters p;
    p.m = 2.0643076923;
    p.J = Vector3(0.02383948, 0.02394241, 0.04399995).asDiagonal();
    p.kx = 4.0 * p.m;
    p.kv = 3.2 * p.m;
    p.kR = 1.0;
    p.kOmega = 1.0;
    p.attitude_scale = Vector3(1.5, 1.5, 0.35);
    p.rate_scale = Vector3(0.35, 0.35, 0.22);
    return p;
}

struct X500Actuators
{
    double arm = 0.174;           // FRD x/y offset, m
    double moment_ratio = 0.016;  // reaction torque / thrust, m
    double motor_constant = 8.54858e-6; // N/(rad/s)^2
    double omega_min = 150.0;
    double omega_max = 1000.0;
    // Must match the PX4 output driver's effective thrust model (normally 0 for gz).
    double thrust_factor = 0.0;

    void validate() const
    {
        if (!std::isfinite(arm) || arm <= 0 || !std::isfinite(moment_ratio) ||
            moment_ratio <= 0 || !std::isfinite(motor_constant) || motor_constant <= 0 ||
            !std::isfinite(omega_min) || omega_min < 0 || !std::isfinite(omega_max) ||
            omega_max <= omega_min || !std::isfinite(thrust_factor) ||
            thrust_factor < 0 || thrust_factor > 1)
            throw std::invalid_argument("Invalid X500 actuator parameters");
    }
    double min_thrust() const { return motor_constant * omega_min * omega_min; }
    double max_thrust() const { return motor_constant * omega_max * omega_max; }

    double command_from_thrust(double thrust) const
    {
        validate();
        if (!std::isfinite(thrust)) throw std::invalid_argument("Non-finite rotor thrust");
        const double speed = std::sqrt(std::clamp(thrust, min_thrust(), max_thrust()) / motor_constant);
        const double u = std::clamp((speed - omega_min) / (omega_max - omega_min), 0.0, 1.0);
        return (1.0 - thrust_factor) * u + thrust_factor * u * u;
    }
};

struct X500Allocation
{
    std::array<double, 4> thrusts{};
    std::array<double, 4> commands{};
    double achieved_f = 0;
    Vector3 achieved_M = Vector3::Zero();
    double roll_pitch_scale = 1;
    bool saturated = false;
};

inline Eigen::Vector4d x500_wrench(const std::array<double, 4> &t,
                                 const X500Actuators &a = {})
{
    // Motor order 0,1,2,3 from the standard PX4 gz_x500 model, body FRD.
    return Eigen::Vector4d(t[0]+t[1]+t[2]+t[3],
        a.arm*(-t[0]+t[1]+t[2]-t[3]),
        a.arm*(t[0]-t[1]+t[2]-t[3]),
        a.moment_ratio*(t[0]+t[1]-t[2]-t[3]));
}

inline X500Allocation allocate_x500(double f, const Vector3 &moment,
                                    const X500Actuators &a = {})
{
    a.validate();
    if (!std::isfinite(f) || !moment.allFinite())
        throw std::invalid_argument("Non-finite requested wrench");
    const double lo = a.min_thrust(), hi = a.max_thrust();
    const double collective = std::clamp(f / 4.0, lo, hi);
    const double b = -moment.x() / (4.0*a.arm);
    const double c = moment.y() / (4.0*a.arm);
    const std::array<double, 4> rp{{b+c, -b-c, -b+c, b-c}};
    for (double value : rp)
        if (!std::isfinite(value)) throw std::invalid_argument("Requested torque overflows allocation");
    X500Allocation out;
    // Preserve collective first, roll/pitch second, yaw last. Saturation never
    // invalidates a finite command or stops the Offboard heartbeat.
    for (double delta : rp)
    {
        if (delta > 0) out.roll_pitch_scale = std::min(out.roll_pitch_scale, (hi-collective)/delta);
        if (delta < 0) out.roll_pitch_scale = std::min(out.roll_pitch_scale, (lo-collective)/delta);
    }
    double yaw_lo = -hi, yaw_hi = hi;
    for (int i = 0; i < 4; ++i)
    {
        out.thrusts[i] = collective + out.roll_pitch_scale*rp[i];
        if (i < 2)
        {
            yaw_lo = std::max(yaw_lo, lo-out.thrusts[i]);
            yaw_hi = std::min(yaw_hi, hi-out.thrusts[i]);
        }
        else
        {
            yaw_lo = std::max(yaw_lo, out.thrusts[i]-hi);
            yaw_hi = std::min(yaw_hi, out.thrusts[i]-lo);
        }
    }
    // Zero is feasible by construction; include it despite rounding at a bound.
    yaw_lo = std::min(yaw_lo, 0.0);
    yaw_hi = std::max(yaw_hi, 0.0);
    const double yaw = std::clamp(moment.z()/(4*a.moment_ratio), yaw_lo, yaw_hi);
    for (int i = 0; i < 4; ++i)
    {
        out.thrusts[i] = std::clamp(out.thrusts[i] + (i < 2 ? yaw : -yaw), lo, hi);
        out.commands[i] = a.command_from_thrust(out.thrusts[i]);
    }
    const auto achieved = x500_wrench(out.thrusts, a);
    out.achieved_f = achieved[0];
    out.achieved_M = achieved.tail<3>();
    out.saturated = std::abs(out.achieved_f-f) > 1e-8 || (out.achieved_M-moment).norm() > 1e-8;
    return out;
}
} // namespace uav_research
