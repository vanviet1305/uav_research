#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>

#include "uav_research/p01_reference.hpp"

namespace uav_research
{

using Vector3 = Eigen::Vector3d;
using Matrix3 = Eigen::Matrix3d;

// State convention of P01: world z points down; body e3 points down.
// R maps body-frame vectors into the world frame.
struct P01State
{
    Vector3 x = Vector3::Zero();
    Vector3 v = Vector3::Zero();
    Matrix3 R = Matrix3::Identity();
    Vector3 omega = Vector3::Zero();
};

// Tham số vật lý và hệ số phản hồi: m (kg), g (m/s²), J (kg.m²).
// Các node X500 ghi đè một số giá trị mặc định này trong constructor.
struct P01Parameters
{
    double m = 4.34;
    double g = 9.81;
    double kx = 16.0 * 4.34;
    double kv = 5.6 * 4.34;
    double kR = 8.81;
    double kOmega = 2.54;
    // Per-axis multipliers; ones preserve the original paper/offline controller.
    Vector3 attitude_scale = Vector3::Ones();
    Vector3 rate_scale = Vector3::Ones();
    Matrix3 J = Vector3(0.0820, 0.0845, 0.1377).asDiagonal();
};

// Kết quả điều khiển kèm các đại lượng phục vụ kiểm tra và ghi log.
// ex/ev: sai số vị trí/vận tốc; eR/eOmega: sai số tư thế/vận tốc góc.
// Rd, omega_d, omega_d_dot: tư thế, vận tốc góc và gia tốc góc mong muốn.
// Psi là hàm sai số tư thế vô hướng; không phải góc tính bằng radian.
struct P01Control
{
    double f = 0.0;              // Total thrust, newtons (ideal, unsaturated).
    Vector3 M = Vector3::Zero(); // Body moment, newton-metres.
    Vector3 ex = Vector3::Zero();
    Vector3 ev = Vector3::Zero();
    Vector3 eR = Vector3::Zero();
    Vector3 eOmega = Vector3::Zero();
    Vector3 A = Vector3::Zero();
    Matrix3 Rd = Matrix3::Identity();
    Vector3 omega_d = Vector3::Zero();
    Vector3 omega_d_dot = Vector3::Zero();
    double Psi = 0.0;
};

// Toán tử hat: hat3(a) * b = a × b; vee3 lấy vector từ phần phản đối xứng.
inline Matrix3 hat3(const Vector3 &a)
{
    Matrix3 S;
    S << 0.0, -a.z(), a.y(), a.z(), 0.0, -a.x(), -a.y(), a.x(), 0.0;
    return S;
}

inline Vector3 vee3(const Matrix3 &S)
{
    const Matrix3 skew = 0.5 * (S - S.transpose());
    return Vector3(skew(2, 1), skew(0, 2), skew(1, 0));
}

// Vector đơn vị và hai đạo hàm, dùng để dựng các trục của tư thế mong muốn.
struct UnitVectorKinematics
{
    Vector3 q, q_dot, q_ddot;
};

inline UnitVectorKinematics normalized_with_derivatives(const Vector3 &u, const Vector3 &u_dot,
                                                        const Vector3 &u_ddot)
{
    const double n = u.norm();
    if (!(n > 1e-9))
    {
        throw std::runtime_error("P01: cannot normalize a near-zero vector");
    }

    UnitVectorKinematics out;
    out.q = u / n;
    const double n_dot = out.q.dot(u_dot);
    const double n_ddot = (u_dot.squaredNorm() + u.dot(u_ddot) - n_dot * n_dot) / n;
    out.q_dot = (u_dot - out.q * n_dot) / n;
    out.q_ddot = (u_ddot - out.q * n_ddot - 2.0 * out.q_dot * n_dot) / n;
    return out;
}

// Direct C++ port of p01_trajectory_controller.m (including feedforward).
inline P01Control compute_p01_control_with_reference(const P01State &state, const P01Parameters &p,
                                                     const P01Reference &r)
{

    if (!(p.m > 0.0))
    {
        throw std::invalid_argument("P01 mass must be positive");
    }

    const Vector3 e3 = Vector3::UnitZ();
    P01Control out;

    // Position loop: e_x, e_v, desired force A, collective thrust f.
    out.ex = state.x - r.x;
    out.ev = state.v - r.v;
    out.A = -p.kx * out.ex - p.kv * out.ev - p.m * p.g * e3 + p.m * r.a;
    const Vector3 b = state.R * e3;
    out.f = -out.A.dot(b);

    // Exact derivatives used in the MATLAB implementation.
    const Vector3 acceleration = p.g * e3 - (out.f / p.m) * b;
    const Vector3 A_dot = -p.kx * out.ev - p.kv * (acceleration - r.a) + p.m * r.j;
    const Matrix3 omega_hat = hat3(state.omega);
    const Vector3 b_dot = state.R * omega_hat * e3;
    const double f_dot = -A_dot.dot(b) - out.A.dot(b_dot);
    const Vector3 jerk = -(f_dot * b + out.f * b_dot) / p.m;
    const Vector3 A_ddot = -p.kx * (acceleration - r.a) - p.kv * (jerk - r.j) + p.m * r.snap;

    const auto b3 = normalized_with_derivatives(-out.A, -A_dot, -A_ddot);

    // Heading preference gives b2 = normalize(b3 x b1_preferred).
    const Vector3 c2 = b3.q.cross(r.b1);
    const Vector3 c2_dot = b3.q_dot.cross(r.b1) + b3.q.cross(r.b1_dot);
    const Vector3 c2_ddot =
        b3.q_ddot.cross(r.b1) + 2.0 * b3.q_dot.cross(r.b1_dot) + b3.q.cross(r.b1_ddot);
    const auto b2 = normalized_with_derivatives(c2, c2_dot, c2_ddot);

    const Vector3 b1 = b2.q.cross(b3.q);
    const Vector3 b1_dot = b2.q_dot.cross(b3.q) + b2.q.cross(b3.q_dot);
    const Vector3 b1_ddot =
        b2.q_ddot.cross(b3.q) + 2.0 * b2.q_dot.cross(b3.q_dot) + b2.q.cross(b3.q_ddot);

    out.Rd.col(0) = b1;
    out.Rd.col(1) = b2.q;
    out.Rd.col(2) = b3.q;

    Matrix3 Rd_dot, Rd_ddot;
    Rd_dot.col(0) = b1_dot;
    Rd_dot.col(1) = b2.q_dot;
    Rd_dot.col(2) = b3.q_dot;
    Rd_ddot.col(0) = b1_ddot;
    Rd_ddot.col(1) = b2.q_ddot;
    Rd_ddot.col(2) = b3.q_ddot;

    out.omega_d = vee3(out.Rd.transpose() * Rd_dot);
    out.omega_d_dot = vee3(Rd_dot.transpose() * Rd_dot + out.Rd.transpose() * Rd_ddot);

    const Matrix3 T = state.R.transpose() * out.Rd;
    out.eR = 0.5 * vee3(out.Rd.transpose() * state.R - state.R.transpose() * out.Rd);
    out.eOmega = state.omega - T * out.omega_d;
    out.M = -p.kR * p.attitude_scale.cwiseProduct(out.eR) -
            p.kOmega * p.rate_scale.cwiseProduct(out.eOmega) + state.omega.cross(p.J * state.omega) -
            p.J * (omega_hat * T * out.omega_d - T * out.omega_d_dot);
    out.Psi = 0.5 * (Matrix3::Identity() - out.Rd.transpose() * state.R).trace();

    if (!std::isfinite(out.f) || !out.M.allFinite())
    {
        throw std::runtime_error("P01 control contains non-finite values");
    }
    return out;
}

// Giu tuong thich voi cac chuong trinh P01 cu
// Mac dinh van su dung quy dao Case I
inline P01Control compute_p01_control(double t, const P01State &state, const P01Parameters &p)
{
    return compute_p01_control_with_reference(state, p, reference_case1(t));
}

} // namespace uav_research
