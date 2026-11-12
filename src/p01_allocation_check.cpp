#include <iostream>
#include <iomanip>
#include "uav_research/x500_control.hpp"

int main()
{
    const auto p = uav_research::x500_parameters();
    for (double yaw_moment : {0.0, -0.307464565949, 1.0})
    {
        const auto a = uav_research::allocate_x500(p.m*p.g, {0, 0, yaw_moment});
        std::cout << std::fixed << std::setprecision(5)
            << "Requested Mz=" << yaw_moment << " achieved=" << a.achieved_M.z()
            << " N.m; collective=" << a.achieved_f << " N; saturated=" << a.saturated << '\n';
        for (int i = 0; i < 4; ++i)
            std::cout << "Motor " << i << ": " << a.thrusts[i] << " N; u=" << a.commands[i] << '\n';
    }
    return 0;
}
