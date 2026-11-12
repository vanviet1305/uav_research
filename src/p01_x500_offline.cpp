// Reduced X500 model for regression testing. NOT PX4/Gazebo and not a paper
// reproduction. Includes bounded allocation, sampled control, one-sample delay,
// first-order rotor-speed response, and 1 ms rigid-body integration.
#include <Eigen/Geometry>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include "uav_research/x500_control.hpp"
#include "uav_research/x500_reference.hpp"

int main(int argc, char **argv)
{
    using namespace uav_research;
    try
    {
        const std::string path = argc > 1 ? argv[1] : "x500_offline.csv";
        const std::string mode = argc > 2 ? argv[2] : "case1";
        const double period = argc > 3 ? std::stod(argv[3]) : 0.01;
        constexpr double dt = .001;
        if ((mode != "hover" && mode != "case1" && mode != "case1_yaw" && mode != "yaw_recovery") ||
            !std::isfinite(period) || period < dt || period > .05)
            throw std::invalid_argument("Usage: p01_x500_offline [csv] [hover|case1|case1_yaw|yaw_recovery] [period_s .001..05]");
        const int control_steps = static_cast<int>(std::lround(period/dt));
        if (std::abs(control_steps*dt-period) > 1e-9) throw std::invalid_argument("period must be a multiple of 1 ms");
        const P01Parameters p = x500_parameters();
        const X500Actuators motors;
        const Vector3 origin(0,0,-3), heading = Vector3::UnitX();
        P01State state;
        state.x = origin+Vector3(.15, -.1, .1);
        const double initial_yaw_deg = mode == "yaw_recovery" ? 60.0 : 2.0;
        state.R = (Eigen::AngleAxisd(initial_yaw_deg*3.141592653589793/180, Vector3::UnitZ())*
                   Eigen::AngleAxisd(5*3.141592653589793/180, Vector3::UnitX())).toRotationMatrix();
        if (mode == "yaw_recovery") state.omega.z() = 0.1;
        std::array<double,4> rotor_speed{}, command_speed{}, pending{};
        rotor_speed.fill(std::sqrt(p.m*p.g/(4*motors.motor_constant)));
        command_speed = pending = rotor_speed;
        const auto reference = [&](double t) {
            return mode == "hover" || mode == "yaw_recovery" ? reference_hover(origin, heading) :
                reference_x500_case1(t-3, origin, heading, .25, 4, 30, mode == "case1_yaw");
        };
        std::mt19937 rng(17);
        std::normal_distribution<double> noise(0,1);
        std::ofstream csv(path);
        if (!csv) throw std::runtime_error("Cannot open CSV");
        csv << "t,x,y,z,xd,yd,zd,position_error,tilt_deg,f_actual,Mz_actual,saturated\n" << std::setprecision(10);
        double peak_error = 0, peak_tilt = 0, sum_squared = 0;
        int samples = 0, saturated_samples = 0;
        bool saturated = false;
        for (int step = 0; step <= 40000; ++step)
        {
            const double t = step*dt;
            const auto r = reference(t);
            if (step % control_steps == 0)
            {
                auto measured = state;
                for (int i = 0; i < 3; ++i)
                {
                    measured.x[i] += .002*noise(rng);
                    measured.v[i] += .01*noise(rng);
                    measured.omega[i] += .002*noise(rng);
                }
                const auto c = compute_p01_control_with_reference(measured, p, r);
                const auto a = allocate_x500(c.f, c.M, motors);
                command_speed = pending; // one entire controller sample of delay
                for (int i = 0; i < 4; ++i) pending[i] = std::sqrt(a.thrusts[i]/motors.motor_constant);
                saturated = a.saturated;
                if (saturated) ++saturated_samples;
            }
            std::array<double,4> thrusts{};
            for (int i = 0; i < 4; ++i)
            {
                const double tau = command_speed[i] > rotor_speed[i] ? .0125 : .025;
                rotor_speed[i] += (1-std::exp(-dt/tau))*(command_speed[i]-rotor_speed[i]);
                thrusts[i] = motors.motor_constant*rotor_speed[i]*rotor_speed[i];
            }
            const auto wrench = x500_wrench(thrusts, motors);
            const double error = (state.x-r.x).norm();
            const double tilt = std::acos(std::clamp(state.R(2,2), -1.0, 1.0))*180/3.141592653589793;
            if (t >= 5) { peak_error = std::max(peak_error, error); sum_squared += error*error; ++samples; }
            peak_tilt = std::max(peak_tilt, tilt);
            if (step % 10 == 0)
                csv << t << ',' << state.x.x() << ',' << state.x.y() << ',' << state.x.z()
                    << ',' << r.x.x() << ',' << r.x.y() << ',' << r.x.z() << ',' << error << ',' << tilt
                    << ',' << wrench[0] << ',' << wrench[3] << ',' << saturated << '\n';
            if (step == 40000) break;
            // Semi-implicit 1 ms rigid-body integration, rotation via exponential map.
            const Vector3 acceleration = p.g*Vector3::UnitZ()-(wrench[0]/p.m)*state.R.col(2);
            const Vector3 angular_acceleration = p.J.ldlt().solve(wrench.tail<3>()-state.omega.cross(p.J*state.omega));
            state.v += dt*acceleration;
            state.x += dt*state.v;
            state.omega += dt*angular_acceleration;
            const double omega = state.omega.norm();
            if (omega > 1e-12) state.R *= Eigen::AngleAxisd(dt*omega, state.omega/omega).toRotationMatrix();
            if (!state.x.allFinite() || !state.R.allFinite() || !state.omega.allFinite() || state.x.norm() > 100)
                throw std::runtime_error("Reduced model diverged");
        }
        const double final_error = (state.x-reference(40).x).norm();
        const double final_psi = compute_p01_control_with_reference(state, p, reference(40)).Psi;
        std::cout << std::fixed << std::setprecision(6) << "REDUCED MODEL (not Gazebo): " << mode
            << " period=" << period << " s; max error after 5s=" << peak_error
            << " m; RMSE=" << std::sqrt(sum_squared/samples) << " m; final=" << final_error
            << " m; final Psi=" << final_psi << "; max tilt=" << peak_tilt
            << " deg; saturated samples=" << saturated_samples << '\n';
        if (peak_error > .2 || final_error > .03 || peak_tilt > 35 || final_psi > .001)
            throw std::runtime_error("Tracking/tilt regression threshold exceeded");
        if (mode == "yaw_recovery" && saturated_samples == 0)
            throw std::runtime_error("Saturation recovery scenario did not exercise saturation");
        return 0;
    }
    catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
