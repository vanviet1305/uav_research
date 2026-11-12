// Mục đích: Điều khiển vị trí qua PX4: gửi setpoint 10 Hz, yêu cầu Offboard và Arm sau giai đoạn gửi ban đầu. Mục tiêu nhận từ /uav/target_position_ned; mặc định [0, 0, -3] m.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp" // Thư viện chứa kiểu thông điệp phản hồi vị trí, vận tốc và hướng của UAV
#include "uav_research/px4_topic.hpp"

class OffboardControl : public rclcpp::Node
{
  public:
    OffboardControl() : Node("offboard_control")
    {
        publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            uav_research::px4_topic<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode"), 10);

        trajectory_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            uav_research::px4_topic<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint"), 10);

        command_publisher_ =
            this->create_publisher<px4_msgs::msg::VehicleCommand>(
                uav_research::px4_topic<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command"), 10);

        // Tạo Subscriber để nhận trạng thái thực tế của UAV từ PX4.
        odometry_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            uav_research::px4_topic<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry"), rclcpp::SensorDataQoS(),

            // Callback được gọi khi Subscriber nhận được thông điệp
            [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
            {
                if (msg->pose_frame != px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED) return;
                if (!heading_initialized_)
                {
                    double w = msg->q[0], x = msg->q[1], y = msg->q[2], z = msg->q[3];
                    const double norm = std::sqrt(w*w+x*x+y*y+z*z);
                    if (!std::isfinite(norm) || norm < 1e-6) return;
                    w /= norm; x /= norm; y /= norm; z /= norm;
                    target_yaw_ = static_cast<float>(std::atan2(2*(w*z+x*y), 1-2*(y*y+z*z)));
                    heading_initialized_ = true;
                }
                if (!std::isfinite(msg->position[0]) || !std::isfinite(msg->position[1]) ||
                    !std::isfinite(msg->position[2]))
                {
                    return;
                }

                // Tính sai số theo mục tiêu đang được gửi cho PX4.
                const float dx = target_x_ - msg->position[0];
                const float dy = target_y_ - msg->position[1];
                const float dz = target_z_ - msg->position[2];
                const float error = std::sqrt(dx * dx + dy * dy + dz * dz);

                if (error < 0.2f)
                {
                    if (!reached_target_)
                    {
                        reached_target_ = true;
                        RCLCPP_INFO(this->get_logger(),
                                    "UAV da den vi tri mong muon! Sai so: %.2f m", error);
                    }
                }
                else
                {
                    reached_target_ = false;
                }

                // Hiển thị vị trí thực tế và sai số, tối đa một lần mỗi giây.
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "UAV position: x=%.2f, y=%.2f, z=%.2f | Error=%.2f m",
                                     msg->position[0], msg->position[1], msg->position[2], error);
            });
        target_subscriber_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/uav/target_position_ned", 10,

            [this](const geometry_msgs::msg::Point::SharedPtr msg)
            {
                const float x = static_cast<float>(msg->x);
                const float y = static_cast<float>(msg->y);
                const float z = static_cast<float>(msg->z);
                if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
                {
                    RCLCPP_WARN(this->get_logger(), "Ignoring non-finite target position");
                    return;
                }

                if (x != target_x_ || y != target_y_ || z != target_z_)
                {
                    reached_target_ = false;
                }
                target_x_ = x;
                target_y_ = y;
                target_z_ = z;

                RCLCPP_INFO(this->get_logger(), "New target: x=%.2f, y=%.2f, z=%.2f", target_x_,
                            target_y_, target_z_);
            });

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]()
            {
                if (this->count_publishers(publisher_->get_topic_name()) > 1)
                    publisher_conflict_ = true;
                if (publisher_conflict_)
                {
                    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "Another Offboard publisher detected; stopped. Select Hold and restart one controller only.");
                    return;
                }
                if (!heading_initialized_) return;
                this->publish_offboard_mode();
                this->publish_trajectory_setpoint();

                // Gửi setpoint trước khi yêu cầu chuyển sang Offboard và Arm.
                if (offboard_setpoint_counter_ == 10)
                {
                    this->publish_vehicle_command(
                        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);

                    this->publish_vehicle_command(
                        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f,
                        0.0f);
                }

                // Dừng bộ đếm ở 11 sau khi đã gửi lệnh.
                if (offboard_setpoint_counter_ < 11)
                {
                    offboard_setpoint_counter_++;
                }
            });
    }

  private:
    float target_x_{0.0f};
    float target_y_{0.0f};
    float target_z_{-3.0f};
    float target_yaw_{0.0f};
    bool heading_initialized_{false};
    bool publisher_conflict_{false};
    // Đánh dấu UAV đã đến vị trí mong muốn hay chưa.
    bool reached_target_{false};
    void publish_offboard_mode()
    {
        px4_msgs::msg::OffboardControlMode msg{};

        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        msg.position = true;

        publisher_->publish(msg);
    }

    void publish_trajectory_setpoint()
    {
        px4_msgs::msg::TrajectorySetpoint msg{};

        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        // Hệ tọa độ NED: Z âm là vị trí cao hơn gốc tọa độ.
        msg.position = {target_x_, target_y_, target_z_};

        // NaN biểu thị các đại lượng không được chỉ định.
        const float nan = std::numeric_limits<float>::quiet_NaN();

        msg.velocity = {nan, nan, nan};
        msg.acceleration = {nan, nan, nan};
        msg.jerk = {nan, nan, nan};
        msg.yaw = target_yaw_;
        msg.yawspeed = nan;

        trajectory_publisher_->publish(msg);
    }

    void publish_vehicle_command(std::uint32_t command, float param1, float param2)
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

        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        command_publisher_->publish(msg);
    }

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    // Subscriber nhận dữ liệu vị trí và trạng thái thực tế của UAV từ PX4.
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_subscriber_;

    std::uint32_t offboard_setpoint_counter_{0};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<OffboardControl>());

    rclcpp::shutdown();

    return 0;
}
