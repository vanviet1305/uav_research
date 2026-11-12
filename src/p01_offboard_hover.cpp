#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <Eigen/Geometry>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include "uav_research/offboard_session.hpp"
#include "uav_research/px4_topic.hpp"
#include "uav_research/x500_control.hpp"
#include "uav_research/x500_reference.hpp"

// PX4 SITL + standard Gazebo X500 only. QGC takes off and holds first.
// Dry-run by default. One external controller at a time; no automatic arming.
class P01OffboardHover : public rclcpp::Node
{
    using Clock = std::chrono::steady_clock;
    using Session = uav_research::OffboardSession;
    using Odom = px4_msgs::msg::VehicleOdometry;
    using Status = px4_msgs::msg::VehicleStatus;
    using Command = px4_msgs::msg::VehicleCommand;
    using Ack = px4_msgs::msg::VehicleCommandAck;
    using Mode = px4_msgs::msg::OffboardControlMode;
    using Motors = px4_msgs::msg::ActuatorMotors;
public:
    P01OffboardHover() : Node("p01_offboard_hover")
    {
        enabled_ = parameter<bool>("enable_actuation", false);
        mode_ = parameter<std::string>("reference_mode", "hover");
        if (mode_ != "hover" && mode_ != "case1")
            throw std::invalid_argument("reference_mode must be hover or case1");
        speed_ = positive("trajectory_speed", 0.25);
        ramp_ = positive("trajectory_ramp_s", 4.0);
        duration_ = positive("trajectory_duration_s", 30.0);
        settle_ = positive("settle_time_s", 3.0);
        rotate_heading_ = parameter<bool>("trajectory_rotate_heading", false);
        if (duration_ < 2*ramp_) throw std::invalid_argument("duration must be >= 2*ramp");
        min_height_ = positive("minimum_takeover_height_m", 2.0);
        parameters_ = uav_research::x500_parameters();
        parameters_.m = positive("mass_kg", parameters_.m);
        parameters_.kx = positive("position_gain_per_mass", 4.0)*parameters_.m;
        parameters_.kv = positive("velocity_gain_per_mass", 3.2)*parameters_.m;
        parameters_.attitude_scale = vector_parameter("attitude_gains", parameters_.attitude_scale);
        parameters_.rate_scale = vector_parameter("rate_gains", parameters_.rate_scale);
        parameters_.J = vector_parameter("inertia_diagonal", parameters_.J.diagonal()).asDiagonal();
        actuators_.arm = positive("rotor_arm_m", actuators_.arm);
        actuators_.moment_ratio = positive("rotor_moment_ratio", actuators_.moment_ratio);
        actuators_.motor_constant = positive("motor_constant", actuators_.motor_constant);
        actuators_.omega_min = parameter<double>("motor_speed_min", 150.0);
        actuators_.omega_max = parameter<double>("motor_speed_max", 1000.0);
        actuators_.thrust_factor = parameter<double>("motor_thrust_factor", 0.0);
        actuators_.validate();
        if (parameters_.m*parameters_.g >= 4*actuators_.max_thrust())
            throw std::invalid_argument("Configured motors cannot support vehicle weight");

        const auto odom_topic = uav_research::px4_topic<Odom>("/fmu/out/vehicle_odometry");
        const auto status_topic = uav_research::px4_topic<Status>("/fmu/out/vehicle_status");
        const auto command_topic = uav_research::px4_topic<Command>("/fmu/in/vehicle_command");
        if (enabled_)
        {
            motor_pub_ = create_publisher<Motors>(
                uav_research::px4_topic<Motors>("/fmu/in/actuator_motors"), rclcpp::SensorDataQoS());
            mode_pub_ = create_publisher<Mode>(
                uav_research::px4_topic<Mode>("/fmu/in/offboard_control_mode"), 10);
            command_pub_ = create_publisher<Command>(command_topic, 10);
        }
        odom_sub_ = create_subscription<Odom>(odom_topic, rclcpp::SensorDataQoS(),
            [this](Odom::SharedPtr msg) { odometry(*msg); });
        status_sub_ = create_subscription<Status>(status_topic, rclcpp::SensorDataQoS(),
            [this](Status::SharedPtr msg)
            {
                status_ = *msg;
                status_time_ = Clock::now();
                have_status_ = true;
                const auto actions = advance();
                if (actions.motors) publish_motors();
            });
        ack_sub_ = create_subscription<Ack>(
            uav_research::px4_topic<Ack>("/fmu/out/vehicle_command_ack"),
            rclcpp::SensorDataQoS(), [this](Ack::SharedPtr msg)
            {
                if (msg->command == Command::VEHICLE_CMD_DO_SET_MODE)
                    RCLCPP_INFO(get_logger(), "Mode ACK result=%u (0=accepted); status confirms ownership",
                        static_cast<unsigned>(msg->result));
            });
        const auto csv = parameter<std::string>("log_csv", "");
        if (!csv.empty())
        {
            log_.open(csv);
            if (!log_) throw std::runtime_error("Cannot open log_csv: "+csv);
            log_ << "t,x,y,z,xd,yd,zd,position_error,Psi,f_requested,f_achieved,"
                    "Mx,My,Mz,Mx_achieved,My_achieved,Mz_achieved,u0,u1,u2,u3,saturated,stage\n";
            log_ << std::setprecision(10);
        }
        timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]()
        {
            if (enabled_)
                exclusive_ = count_publishers(mode_pub_->get_topic_name()) <= 1 &&
                             count_publishers(motor_pub_->get_topic_name()) <= 1;
            const auto actions = advance();
            if (actions.heartbeat)
            {
                Mode msg{};
                msg.timestamp = timestamp_us();
                msg.direct_actuator = true;
                mode_pub_->publish(msg);
            }
            if (session_.stage() == Session::Stage::Waiting || !enabled_)
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                    "Waiting/dry-run: status=%d odom=%d valid=%d rate_ok=%d stable=%d enabled=%d",
                    have_status_, have_odom_, motor_valid_, rate_ok_, stable_airborne(), enabled_);
            if (log_) log_.flush();
        });
        RCLCPP_INFO(get_logger(), "X500 %s, reference=%s. Take off in QGC, select Hold; no auto-arm.",
            enabled_ ? "ACTUATION ENABLED" : "DRY-RUN", mode_.c_str());
        RCLCPP_INFO(get_logger(), "Topics: odom=%s status=%s command=%s", odom_topic.c_str(),
            status_topic.c_str(), command_topic.c_str());
    }
private:
    template<typename T> T parameter(const std::string &name, const T &value)
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = true; // validated startup overrides; no silent in-flight retuning
        return declare_parameter<T>(name, value, descriptor);
    }
    double positive(const std::string &name, double value)
    {
        const double out = parameter<double>(name, value);
        if (!std::isfinite(out) || out <= 0) throw std::invalid_argument(name+" must be finite and >0");
        return out;
    }
    Eigen::Vector3d vector_parameter(const std::string &name, const Eigen::Vector3d &value)
    {
        const auto v = parameter<std::vector<double>>(name, {value.x(), value.y(), value.z()});
        if (v.size() != 3) throw std::invalid_argument(name+" needs 3 values");
        const Eigen::Vector3d out(v[0], v[1], v[2]);
        if (!out.allFinite() || out.minCoeff() <= 0) throw std::invalid_argument(name+" must be positive");
        return out;
    }
    std::uint64_t timestamp_us() { return get_clock()->now().nanoseconds()/1000; }
    double wall_seconds() const { return std::chrono::duration<double>(Clock::now()-epoch_).count(); }
    bool stable_airborne() const
    {
        const bool holding = have_status_ && (status_.nav_state == Status::NAVIGATION_STATE_AUTO_LOITER ||
            status_.nav_state == Status::NAVIGATION_STATE_POSCTL);
        return holding && have_odom_ && state_.x.z() < -min_height_ && state_.v.norm() < 0.3 &&
            state_.omega.norm() < 0.3 && state_.R(2,2) > 0.98;
    }
    bool feedback_ok() const
    {
        const auto now = Clock::now();
        return have_status_ && have_odom_ && motor_valid_ && rate_ok_ &&
            std::chrono::duration<double>(now-status_time_).count() < 1.0 &&
            std::chrono::duration<double>(now-odom_time_).count() < 0.15;
    }
    uav_research::OffboardActions advance()
    {
        uav_research::OffboardInputs in;
        in.enabled = enabled_;
        in.feedback_ok = feedback_ok();
        in.armed = have_status_ && status_.arming_state == Status::ARMING_STATE_ARMED;
        in.failsafe = have_status_ && status_.failsafe;
        in.offboard = have_status_ && status_.nav_state == Status::NAVIGATION_STATE_OFFBOARD;
        in.stable_airborne = stable_airborne();
        in.exclusive = exclusive_;
        const auto before = session_.stage();
        const auto actions = session_.step(wall_seconds(), in);
        if (before != session_.stage())
        {
            RCLCPP_INFO(get_logger(), "Session stage=%d: %s", static_cast<int>(session_.stage()), session_.reason());
            if (session_.stage() == Session::Stage::Active) active_sample_ = last_sample_;
        }
        if (actions.request_offboard) command_mode(6, 0);
        if (actions.request_hold) command_mode(4, 3);
        return actions;
    }
    void command_mode(float mode, float submode)
    {
        Command msg{};
        msg.timestamp = timestamp_us();
        msg.command = Command::VEHICLE_CMD_DO_SET_MODE;
        msg.param1 = 1;
        msg.param2 = mode;
        msg.param3 = submode;
        msg.target_system = msg.source_system = 1;
        msg.target_component = msg.source_component = 1;
        msg.from_external = true;
        command_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "Requested PX4 mode %.0f / %.0f", mode, submode);
    }
    void publish_motors()
    {
        Motors msg{};
        msg.timestamp = timestamp_us();
        msg.timestamp_sample = last_sample_;
        msg.control.fill(std::numeric_limits<float>::quiet_NaN());
        for (int i = 0; i < 4; ++i) msg.control[i] = allocation_.commands[i];
        motor_pub_->publish(msg);
    }
    void odometry(const Odom &msg)
    {
        motor_valid_ = false;
        if (msg.pose_frame != Odom::POSE_FRAME_NED || msg.velocity_frame != Odom::VELOCITY_FRAME_NED)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Expected NED pose and velocity frames");
            advance();
            return;
        }
        uav_research::P01State next;
        next.x = Eigen::Vector3d(msg.position[0], msg.position[1], msg.position[2]);
        next.v = Eigen::Vector3d(msg.velocity[0], msg.velocity[1], msg.velocity[2]);
        next.omega = Eigen::Vector3d(msg.angular_velocity[0], msg.angular_velocity[1], msg.angular_velocity[2]);
        Eigen::Quaterniond q(msg.q[0], msg.q[1], msg.q[2], msg.q[3]);
        if (!next.x.allFinite() || !next.v.allFinite() || !next.omega.allFinite() ||
            !q.coeffs().allFinite() || q.norm() < 1e-6) { advance(); return; }
        const auto sample = msg.timestamp_sample != 0 ? msg.timestamp_sample : msg.timestamp;
        if (sample == 0 || (have_odom_ && sample <= last_sample_)) { advance(); return; }
        // Recommended 50-100 Hz. A gap >50 ms is treated as invalid feedback.
        rate_ok_ = have_odom_ && (sample-last_sample_) <= 50000;
        if (have_odom_ && msg.reset_counter != reset_counter_)
        {
            have_odom_ = false;
            advance();
            return;
        }
        reset_counter_ = msg.reset_counter;
        q.normalize();
        next.R = q.toRotationMatrix();
        state_ = next;
        last_sample_ = sample;
        have_odom_ = true;
        odom_time_ = Clock::now();
        const auto stage = session_.stage();
        if (!anchor_set_ || (enabled_ && (stage == Session::Stage::Waiting || stage == Session::Stage::Priming)))
        {
            origin_ = state_.x;
            heading_ = uav_research::reference_hover(origin_, state_.R.col(0)).b1;
            anchor_set_ = true;
        }
        const double t = active_sample_ == 0 || sample < active_sample_ ? 0 : (sample-active_sample_)*1e-6;
        try
        {
            const auto ref = mode_ == "case1" && active_sample_ != 0 ?
                uav_research::reference_x500_case1(t-settle_, origin_, heading_, speed_, ramp_, duration_, rotate_heading_) :
                uav_research::reference_hover(origin_, heading_);
            const auto control = uav_research::compute_p01_control_with_reference(state_, parameters_, ref);
            allocation_ = uav_research::allocate_x500(control.f, control.M, actuators_);
            motor_valid_ = true;
            const auto actions = advance();
            if (actions.motors) publish_motors();
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "t=%.2f ex=%.3f Psi=%.4f f=%.2f/%.2f Mz=%.3f/%.3f saturated=%d motor_gate=%s",
                t, control.ex.norm(), control.Psi, control.f, allocation_.achieved_f,
                control.M.z(), allocation_.achieved_M.z(), allocation_.saturated,
                actions.motors ? "ACTIVE" : "BLOCKED");
            if (log_)
            {
                log_ << t;
                for (int i = 0; i < 3; ++i) log_ << ',' << state_.x[i];
                for (int i = 0; i < 3; ++i) log_ << ',' << ref.x[i];
                log_ << ',' << control.ex.norm() << ',' << control.Psi << ',' << control.f << ',' << allocation_.achieved_f;
                for (int i = 0; i < 3; ++i) log_ << ',' << control.M[i];
                for (int i = 0; i < 3; ++i) log_ << ',' << allocation_.achieved_M[i];
                for (double u : allocation_.commands) log_ << ',' << u;
                log_ << ',' << allocation_.saturated << ',' << static_cast<int>(session_.stage()) << '\n';
            }
        }
        catch (const std::exception &e)
        {
            motor_valid_ = false;
            RCLCPP_ERROR(get_logger(), "Control error: %s", e.what());
            advance();
        }
    }
    rclcpp::Subscription<Odom>::SharedPtr odom_sub_;
    rclcpp::Subscription<Status>::SharedPtr status_sub_;
    rclcpp::Subscription<Ack>::SharedPtr ack_sub_;
    rclcpp::Publisher<Motors>::SharedPtr motor_pub_;
    rclcpp::Publisher<Mode>::SharedPtr mode_pub_;
    rclcpp::Publisher<Command>::SharedPtr command_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    uav_research::P01Parameters parameters_;
    uav_research::X500Actuators actuators_;
    uav_research::X500Allocation allocation_;
    uav_research::P01State state_;
    Session session_;
    Status status_{};
    Clock::time_point epoch_ = Clock::now(), odom_time_{}, status_time_{};
    bool enabled_ = false, have_status_ = false, have_odom_ = false, motor_valid_ = false;
    bool rate_ok_ = false, exclusive_ = true, anchor_set_ = false, rotate_heading_ = false;
    std::uint64_t last_sample_ = 0, active_sample_ = 0;
    std::uint8_t reset_counter_ = 0;
    double speed_ = 0.25, ramp_ = 4, duration_ = 30, settle_ = 3, min_height_ = 2;
    std::string mode_;
    Eigen::Vector3d origin_ = Eigen::Vector3d::Zero(), heading_ = Eigen::Vector3d::UnitX();
    std::ofstream log_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try { rclcpp::spin(std::make_shared<P01OffboardHover>()); }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("p01_offboard_hover"), "%s", e.what());
        if (rclcpp::ok()) rclcpp::shutdown();
        return 1;
    }
    if (rclcpp::ok()) rclcpp::shutdown();
    return 0;
}
