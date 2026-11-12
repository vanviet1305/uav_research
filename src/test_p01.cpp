// Mục đích: In các đại lượng tham chiếu Case I tại t = 0 để kiểm tra bằng mắt; không có assertion tự động.

#include <iostream>
#include "uav_research/p01_reference.hpp"

int main()
{
    double t = 0.0;

    // Gọi hàm sinh quỹ đạo mong muốn của P01
    uav_research::P01Reference ref = uav_research::reference_case1(t);

    // In kết quả
    std::cout << "Thoi gian: " << t << " s\n";

    std::cout << "x_d: " << ref.x.transpose() << "\n";

    std::cout << "v_d: " << ref.v.transpose() << "\n";

    std::cout << "a_d: " << ref.a.transpose() << "\n";

    std::cout << "b1_d: " << ref.b1.transpose() << "\n";

    return 0;
}