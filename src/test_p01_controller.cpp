// Mục đích: Kiểm tra bộ điều khiển tại trạng thái đầu Case I: đối chiếu sai số, lực và tính trực giao của Rd; trả mã lỗi nếu không đạt.

#include <iomanip>
#include <iostream>
#include <exception>
#include <cmath>

#include "uav_research/p01_geometric_controller.hpp"

int main()
{
    try
    {
        const uav_research::P01Parameters p;
        const uav_research::P01State initial; // MATLAB Case I: all zeros, R=I.
        const auto c = uav_research::compute_p01_control(0.0, initial, p);

        std::cout << std::fixed << std::setprecision(6);
        std::cout << "P01 Case I, t = 0 s\n";
        std::cout << "ex      = " << c.ex.transpose() << '\n';
        std::cout << "ev      = " << c.ev.transpose() << '\n';
        std::cout << "A       = " << c.A.transpose() << " N\n";
        std::cout << "f       = " << c.f << " N\n";
        std::cout << "M       = " << c.M.transpose() << " N.m\n";
        std::cout << "eR      = " << c.eR.transpose() << '\n';
        std::cout << "eOmega  = " << c.eOmega.transpose() << '\n';
        std::cout << "Psi     = " << c.Psi << '\n';
        std::cout << "Rd^T Rd orthogonality error = "
                  << (c.Rd.transpose() * c.Rd - uav_research::Matrix3::Identity()).norm() << '\n';

        // Independent checks derived from the MATLAB Case I initial state.
        const uav_research::Vector3 expected_ex(0.0, 0.0, -0.6);
        const uav_research::Vector3 expected_ev(-0.4, -1.256, 0.0);
        const double expected_f = 26.5857984;
        if ((c.ex - expected_ex).norm() > 1e-9 || (c.ev - expected_ev).norm() > 1e-9 ||
            std::abs(c.f - expected_f) > 1e-6 ||
            (c.Rd.transpose() * c.Rd - uav_research::Matrix3::Identity()).norm() > 1e-9)
        {
            std::cerr << "CHECK FAILED: P01 Case I initial-state values differ\n";
            return 2;
        }
        std::cout << "CHECK PASSED: P01 initial-state values\n";
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "P01 test failed: " << e.what() << '\n';
        return 1;
    }
}