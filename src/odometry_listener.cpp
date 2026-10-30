
#include <memory> //Cung cấp smart pointer như std::shared_ptr và std::make_shared
#include "rclcpp/rclcpp.hpp"//Cung cấp các thành phần lập trình ROS 2 bằng C++
#include "px4_msgs/msg/vehicle_odometry.hpp"//Cung cấp kiểu thông điệp VehicleOdometry của PX4
//px4_msgs không trực tiếp đọc cảm biến. Nó cung cấp các kiểu thông điệp để chương trình C++ hiểu cấu trúc dữ liệu do PX4 gửi đến.
class OdometryListener : public rclcpp::Node // kế thừa, dấu :: thể hiện kế thùa ở trong namespace
{
public:
    OdometryListener() : Node("odometry_listener") //Gọi constructor của class cha, truyền tên node
    {
        subscription_ = //lưu smart pointer đó vào subscription_.
            this->create_subscription<px4_msgs::msg::VehicleOdometry>(//chỉ định kiểu dữ liệu mà Subscription sẽ nhận
                "/fmu/out/vehicle_odometry",
                rclcpp::SensorDataQoS(),
                [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg)// Callback
                {
                    RCLCPP_INFO_THROTTLE(   // ghi log bị giới hạn, thông thường không quá một lần mỗi giây tại vị trí gọi đó.
                        this->get_logger(),
                        *this->get_clock(),
                        1000,
                        "X=%.2f Y=%.2f Z=%.2f",
                        msg->position[0],
                        msg->position[1],
                        msg->position[2]);
                });
    }

private:
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr
        subscription_;
};//Khai báo một biến thành viên tên subscription_, có kiểu smart pointer quản lý Subscription nhận thông điệp VehicleOdometry.

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);//
    rclcpp::spin(std::make_shared<OdometryListener>());
    rclcpp::shutdown();
    return 0;
}




