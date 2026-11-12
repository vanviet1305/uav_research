// Mục đích: Mô phỏng kín P01 Case I không cần PX4: tích phân RK4 bước 1 ms trong 10 s, xuất CSV mỗi 10 bước; hiệu chỉnh ma trận quay bằng SVD.

#include <Eigen/Dense>
#include <Eigen/SVD>

#include <cmath>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include "uav_research/p01_geometric_controller.hpp"

namespace
{

using uav_research::Matrix3;
using uav_research::Vector3;
using uav_research::P01State;
using uav_research::P01Parameters;

struct Derivative
{
    Vector3 dx = Vector3::Zero();
    Vector3 dv = Vector3::Zero();
    Matrix3 dR = Matrix3::Zero();
    Vector3 domega = Vector3::Zero();
};

// P01 plant: xdot=v; m*vdot=mg*e3-f*R*e3;
// Rdot=R*hat(omega); J*omegadot=M-omega x J*omega.
Derivative dynamics(double t, const P01State &s, const P01Parameters &p)
{
    const auto command = uav_research::compute_p01_control(t, s, p);
    Derivative d;
    d.dx = s.v;
    d.dv = p.g * Vector3::UnitZ() - (command.f / p.m) * s.R.col(2);
    d.dR = s.R * uav_research::hat3(s.omega);
    d.domega = p.J.ldlt().solve(command.M - s.omega.cross(p.J * s.omega));
    return d;
}

P01State add_scaled(const P01State &s, const Derivative &d, double h)
{
    P01State result = s;
    result.x += h * d.dx;
    result.v += h * d.dv;
    result.R += h * d.dR;
    result.omega += h * d.domega;
    return result;
}

// Fixed-step RK4. Unlike MATLAB's ode45, this is not an adaptive solver.
P01State rk4_step(double t, double h, const P01State &s, const P01Parameters &p)
{
    const Derivative k1 = dynamics(t, s, p);
    const Derivative k2 = dynamics(t + h / 2.0, add_scaled(s, k1, h / 2.0), p);
    const Derivative k3 = dynamics(t + h / 2.0, add_scaled(s, k2, h / 2.0), p);
    const Derivative k4 = dynamics(t + h, add_scaled(s, k3, h), p);

    P01State next = s;
    next.x += (h / 6.0) * (k1.dx + 2.0 * k2.dx + 2.0 * k3.dx + k4.dx);
    next.v += (h / 6.0) * (k1.dv + 2.0 * k2.dv + 2.0 * k3.dv + k4.dv);
    next.R += (h / 6.0) * (k1.dR + 2.0 * k2.dR + 2.0 * k3.dR + k4.dR);
    next.omega += (h / 6.0) * (k1.domega + 2.0 * k2.domega + 2.0 * k3.domega + k4.domega);

    // Project the numerically integrated R onto SO(3): R^T R = I, det R = 1.
    const Eigen::JacobiSVD<Matrix3> svd(next.R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Matrix3 U = svd.matrixU();
    const Matrix3 V = svd.matrixV();
    Matrix3 D = Matrix3::Identity();
    if ((U * V.transpose()).determinant() < 0.0)
    {
        D(2, 2) = -1.0;
    }
    next.R = U * D * V.transpose();
    return next;
}

void write_csv_header(std::ofstream &csv)
{
    csv << "t,x1,x2,x3,xd1,xd2,xd3,ex_norm,ev_norm,Psi,eOmega_norm,"
           "f,M1,M2,M3,R_orth_error,R_det_error\n";
}

void write_csv_row(std::ofstream &csv, double t, const P01State &s, const P01Parameters &p)
{
    const auto r = uav_research::reference_case1(t);
    const auto c = uav_research::compute_p01_control(t, s, p);
    const double orth_error = (s.R.transpose() * s.R - Matrix3::Identity()).norm();
    const double det_error = std::abs(s.R.determinant() - 1.0);
    csv << t << ',' << s.x.x() << ',' << s.x.y() << ',' << s.x.z() << ',' << r.x.x() << ','
        << r.x.y() << ',' << r.x.z() << ',' << c.ex.norm() << ',' << c.ev.norm() << ',' << c.Psi
        << ',' << c.eOmega.norm() << ',' << c.f << ',' << c.M.x() << ',' << c.M.y() << ','
        << c.M.z() << ',' << orth_error << ',' << det_error << '\n';
}

} // namespace

int main(int argc, char **argv)
{
    try
    {
        const std::string csv_path = argc > 1 ? argv[1] : "p01_case1_cpp.csv";
        std::ofstream csv(csv_path);
        if (!csv)
        {
            throw std::runtime_error("Cannot open CSV output: " + csv_path);
        }
        csv << std::fixed << std::setprecision(10);
        write_csv_header(csv);

        const P01Parameters p;
        P01State s; // MATLAB bai_11a: x0=v0=omega0=0 and R0=I.

        constexpr double dt = 0.001;
        constexpr int steps = 10000;  // 10 seconds.
        constexpr int log_every = 10; // 100 Hz CSV output.

        for (int i = 0; i <= steps; ++i)
        {
            const double t = i * dt;
            if (i % log_every == 0)
            {
                write_csv_row(csv, t, s, p);
            }
            if (i < steps)
            {
                s = rk4_step(t, dt, s, p);
                if (!s.x.allFinite() || !s.v.allFinite() || !s.R.allFinite() ||
                    !s.omega.allFinite())
                {
                    throw std::runtime_error("Non-finite state during simulation");
                }
            }
        }

        const auto final_control = uav_research::compute_p01_control(10.0, s, p);
        std::cout << std::setprecision(8) << "P01 offline Case I finished (RK4, dt=0.001 s).\n"
                  << "CSV: " << csv_path << '\n'
                  << "Final position error (m): " << final_control.ex.norm() << '\n'
                  << "Final attitude Psi: " << final_control.Psi << '\n';
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "P01 offline simulation failed: " << e.what() << '\n';
        return 1;
    }
}
