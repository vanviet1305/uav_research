#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <Eigen/Geometry>
#include <memory>
#include "uav_research/px4_topic.hpp"
#include "uav_research/x500_control.hpp"

// Read-only hover diagnostic using exactly the same default controller and
// allocator as p01_offboard_hover. No publishers, no automatic 10 s shutdown.
class P01ShadowController : public rclcpp::Node
{
    using Odom = px4_msgs::msg::VehicleOdometry;
public:
    P01ShadowController() : Node("p01_shadow_controller")
    {
        sub_ = create_subscription<Odom>(uav_research::px4_topic<Odom>("/fmu/out/vehicle_odometry"),
            rclcpp::SensorDataQoS(), [this](Odom::SharedPtr msg) { observe(*msg); });
        RCLCPP_INFO(get_logger(), "Read-only X500 hover diagnostic; start after QGC takeoff.");
    }
private:
    void observe(const Odom &msg)
    {
        if (msg.pose_frame != Odom::POSE_FRAME_NED || msg.velocity_frame != Odom::VELOCITY_FRAME_NED) return;
        uav_research::P01State s;
        s.x = Eigen::Vector3d(msg.position[0], msg.position[1], msg.position[2]);
        s.v = Eigen::Vector3d(msg.velocity[0], msg.velocity[1], msg.velocity[2]);
        s.omega = Eigen::Vector3d(msg.angular_velocity[0], msg.angular_velocity[1], msg.angular_velocity[2]);
        Eigen::Quaterniond q(msg.q[0], msg.q[1], msg.q[2], msg.q[3]);
        if (!s.x.allFinite() || !s.v.allFinite() || !s.omega.allFinite() ||
            !q.coeffs().allFinite() || q.norm() < 1e-6) return;
        q.normalize();
        s.R = q.toRotationMatrix();
        if (!initialized_ || msg.reset_counter != reset_counter_)
        {
            reference_ = uav_research::reference_hover(s.x, s.R.col(0));
            initialized_ = true;
            reset_counter_ = msg.reset_counter;
        }
        try
        {
            const auto c = uav_research::compute_p01_control_with_reference(s, parameters_, reference_);
            const auto a = uav_research::allocate_x500(c.f, c.M);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "SHADOW ex=%.3f Psi=%.4f f=%.2f/%.2f Mz=%.3f/%.3f saturated=%d u=[%.3f %.3f %.3f %.3f]",
                c.ex.norm(), c.Psi, c.f, a.achieved_f, c.M.z(), a.achieved_M.z(), a.saturated,
                a.commands[0], a.commands[1], a.commands[2], a.commands[3]);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Calculation failed: %s", e.what());
        }
    }
    rclcpp::Subscription<Odom>::SharedPtr sub_;
    uav_research::P01Parameters parameters_ = uav_research::x500_parameters();
    uav_research::P01Reference reference_{};
    bool initialized_ = false;
    std::uint8_t reset_counter_ = 0;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<P01ShadowController>());
    if (rclcpp::ok()) rclcpp::shutdown();
    return 0;
}
