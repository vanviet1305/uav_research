#include <chrono>
#include <memory>
#include <limits>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp" // Thư viện chứa kiểu thông điệp phản hồi vị trí, vận tốc và hướng của UAV

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control")
    {
        publisher_ =
            this->create_publisher<px4_msgs::msg::OffboardControlMode>(
                "/fmu/in/offboard_control_mode",
                10);

        trajectory_publisher_ =
            this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
                "/fmu/in/trajectory_setpoint",
                10);

        command_publisher_ =
            this->create_publisher<px4_msgs::msg::VehicleCommand>(
                "/fmu/in/vehicle_command_v1",
                10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]()
            {
                this->publish_offboard_mode();
                this->publish_trajectory_setpoint();

                // Gửi setpoint trước khi yêu cầu chuyển sang Offboard và Arm.
                if (offboard_setpoint_counter_ == 10)
                {
                    this->publish_vehicle_command(
                        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
                        1.0f,
                        6.0f
                    );

                    this->publish_vehicle_command(
                        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
                        1.0f,
                        0.0f
                    );
                }

                // Dừng bộ đếm ở 11 sau khi đã gửi lệnh.
                if (offboard_setpoint_counter_ < 11)
                {
                    offboard_setpoint_counter_++;
                }
            });
    }

private:
    void publish_offboard_mode()
    {
        px4_msgs::msg::OffboardControlMode msg{};

        msg.timestamp =
            this->get_clock()->now().nanoseconds() / 1000;

        msg.position = true;

        publisher_->publish(msg);
    }

    void publish_trajectory_setpoint()
    {
        px4_msgs::msg::TrajectorySetpoint msg{};

        msg.timestamp =
            this->get_clock()->now().nanoseconds() / 1000;

        // Hệ tọa độ NED: Z âm là vị trí cao hơn gốc tọa độ.
        msg.position = {0.0f, 0.0f, -3.0f};

        // NaN biểu thị các đại lượng không được chỉ định.
        const float nan =
            std::numeric_limits<float>::quiet_NaN();

        msg.velocity = {nan, nan, nan};
        msg.acceleration = {nan, nan, nan};
        msg.jerk = {nan, nan, nan};

        msg.yaw = 0.0f;
        msg.yawspeed = nan;

        trajectory_publisher_->publish(msg);
    }

    void publish_vehicle_command(
        std::uint32_t command,
        float param1,
        float param2)
    {
        px4_msgs::msg::VehicleCommand msg{};

        msg.command = command;
        msg.param1 = param1;
        msg.param2 = param2;

        msg.target_system = 1;
        msg.target_component = 1;

        msg.source_system = 1;
        msg.source_component = 1;

        msg.from_external = true;

        msg.timestamp =
            this->get_clock()->now().nanoseconds() / 1000;

        command_publisher_->publish(msg);
    }

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
        publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr
        trajectory_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr
        command_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::uint32_t offboard_setpoint_counter_{0};
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<OffboardControl>());

    rclcpp::shutdown();

    return 0;
}
