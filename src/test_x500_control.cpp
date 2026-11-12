#include <Eigen/Geometry>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include "uav_research/offboard_session.hpp"
#include "uav_research/px4_topic.hpp"
#include "uav_research/x500_control.hpp"
#include "uav_research/x500_reference.hpp"

using namespace uav_research;
namespace
{
void require(bool ok, const char *message)
{
    if (!ok) throw std::runtime_error(message);
}
void near(double actual, double expected, double tolerance, const char *message)
{
    require(std::isfinite(actual) && std::abs(actual-expected) < tolerance, message);
}
struct Unversioned {};
struct Versioned { static constexpr unsigned MESSAGE_VERSION = 5; };

void allocation_tests()
{
    const auto p = x500_parameters();
    X500Actuators a;
    const double weight = p.m*p.g;
    auto hover = allocate_x500(weight, Vector3::Zero());
    near(hover.achieved_f, weight, 1e-10, "hover weight");
    require(!hover.saturated, "hover must not saturate");
    for (double t : hover.thrusts) near(t, weight/4, 1e-10, "equal hover rotors");
    // Independent physical sign check: torque = rotor position cross upward force.
    const std::array<Vector3,4> positions{{{.174,.174,0},{-.174,-.174,0},{.174,-.174,0},{-.174,.174,0}}};
    for (int rotor = 0; rotor < 4; ++rotor)
    {
        std::array<double,4> impulse{};
        impulse[rotor] = 1;
        const Vector3 mechanical = positions[rotor].cross(Vector3(0,0,-1));
        const auto wrench = x500_wrench(impulse);
        near(wrench[1], mechanical.x(), 1e-12, "physical roll sign/order");
        near(wrench[2], mechanical.y(), 1e-12, "physical pitch sign/order");
        near(wrench[3], rotor < 2 ? .016 : -.016, 1e-12, "physical yaw sign/order");
    }
    // Regression: original gains at a 2 degree yaw error used to stop all output.
    auto yaw = allocate_x500(weight, {0, 0, -8.81*std::sin(2*3.141592653589793/180)});
    require(yaw.saturated, "large yaw must be limited");
    near(yaw.achieved_f, weight, 1e-9, "yaw limiting preserves collective");
    near(yaw.achieved_M.x(), 0, 1e-9, "yaw limiting preserves roll");
    near(yaw.achieved_M.y(), 0, 1e-9, "yaw limiting preserves pitch");
    near(yaw.achieved_M.z(), -0.22309538461659, 1e-9, "yaw capacity");
    auto priority = allocate_x500(weight, {0.2, -0.1, 10.0});
    near(priority.achieved_M.x(), 0.2, 1e-9, "roll takes priority over yaw");
    near(priority.achieved_M.y(), -0.1, 1e-9, "pitch takes priority over yaw");
    std::mt19937 rng(12345);
    std::uniform_real_distribution<double> thrust(-20, 80), torque(-10, 10);
    for (int k = 0; k < 10000; ++k)
    {
        const double f = thrust(rng);
        const Vector3 m(torque(rng), torque(rng), torque(rng));
        auto out = allocate_x500(f, m, a);
        near(out.achieved_f, std::clamp(f, 4*a.min_thrust(), 4*a.max_thrust()), 1e-8,
            "allocation collective conservation");
        near(out.achieved_M.x(), m.x()*out.roll_pitch_scale, 1e-8, "roll scale");
        near(out.achieved_M.y(), m.y()*out.roll_pitch_scale, 1e-8, "pitch scale");
        for (int i = 0; i < 4; ++i)
        {
            require(std::isfinite(out.commands[i]) && out.commands[i] >= 0 && out.commands[i] <= 1,
                "bounded finite motor command");
            require(out.thrusts[i] >= a.min_thrust() && out.thrusts[i] <= a.max_thrust(), "rotor limits");
            const double speed = a.omega_min+(a.omega_max-a.omega_min)*out.commands[i];
            near(a.motor_constant*speed*speed, out.thrusts[i], 1e-9, "motor speed mapping");
        }
    }
    for (double factor : {0.0, 0.5, 1.0})
    {
        a.thrust_factor = factor;
        const double command = a.command_from_thrust(weight/4);
        const double speed_fraction = factor == 0 ? command :
            (-(1-factor)+std::sqrt((1-factor)*(1-factor)+4*factor*command))/(2*factor);
        near(a.motor_constant*std::pow(a.omega_min+(a.omega_max-a.omega_min)*speed_fraction, 2),
            weight/4, 1e-9, "PX4 thrust curve inverse round trip");
    }
    bool rejected = false;
    try { allocate_x500(std::numeric_limits<double>::quiet_NaN(), Vector3::Zero()); }
    catch (const std::invalid_argument &) { rejected = true; }
    require(rejected, "reject NaN");
    P01State s;
    s.x = Vector3(0,0,-3);
    s.R = Eigen::AngleAxisd(2*3.141592653589793/180, Vector3::UnitZ()).toRotationMatrix();
    const auto control = compute_p01_control_with_reference(s, p, reference_hover(s.x, Vector3::UnitX()));
    require(!allocate_x500(control.f, control.M).saturated, "new X500 gains handle 2 degree yaw");
}

void reference_tests()
{
    const Vector3 origin(2, 3, -3), heading(0, 1, 0);
    const auto reference = [&](double t) { return reference_x500_case1(t, origin, heading, .25, 4, 30, true); };
    for (double t : {-1.0, 0.0})
    {
        const auto r = reference(t);
        require((r.x-origin).norm() < 1e-12 && r.v.norm() < 1e-12 && r.a.norm() < 1e-12 &&
            r.j.norm() < 1e-12 && r.snap.norm() < 1e-12, "smooth stationary start");
        require((r.b1-heading).norm() < 1e-12, "initial heading");
    }
    // The fourth derivative is continuous at joins; its slope may jump, so a
    // centered difference there has O(h), rather than O(h^2), truncation error.
    const double h = 1e-6;
    for (double t : {0.3, 2.0, 3.8, 4.0, 10.0, 26.0, 28.0, 29.8, 30.0})
    {
        const auto l = reference(t-h), r = reference(t), u = reference(t+h);
        require(((u.x-l.x)/(2*h)-r.v).norm() < 1e-6, "trajectory velocity derivative");
        require(((u.v-l.v)/(2*h)-r.a).norm() < 1e-6, "trajectory acceleration derivative");
        require(((u.a-l.a)/(2*h)-r.j).norm() < 1e-6, "trajectory jerk derivative");
        const double snap_error = ((u.j-l.j)/(2*h)-r.snap).norm();
        if (snap_error >= 1e-6)
            throw std::runtime_error("trajectory snap derivative at t="+std::to_string(t)+
                " error="+std::to_string(snap_error));
        require(((u.b1-l.b1)/(2*h)-r.b1_dot).norm() < 1e-6, "heading derivative");
        require(((u.b1_dot-l.b1_dot)/(2*h)-r.b1_ddot).norm() < 1e-6, "heading acceleration");
        require(r.x.z() <= origin.z()+1e-10, "trajectory never goes below airborne start");
    }
    const auto end = reference(30), later = reference(40);
    require((end.x-later.x).norm() < 1e-10 && end.v.norm() < 1e-10 &&
        end.a.norm() < 1e-10 && end.j.norm() < 1e-10 && end.snap.norm() < 1e-10, "smooth stop and hold");
}

void session_tests()
{
    using Stage = OffboardSession::Stage;
    OffboardInputs in;
    in.feedback_ok = in.armed = in.stable_airborne = true;
    OffboardSession s;
    require(!s.step(0, in).heartbeat, "dry run never publishes");
    in.enabled = true;
    in.stable_airborne = false;
    require(!s.step(0, in).heartbeat, "do not take over on ground");
    in.stable_airborne = true;
    require(s.step(1, in).heartbeat, "prime heartbeat");
    require(!s.step(2, in).request_offboard, "minimum prestream");
    const auto request = s.step(2.3, in);
    require(request.request_offboard && !request.motors, "request before motors");
    require(!s.step(2.4, in).request_offboard, "single mode request");
    in.offboard = true;
    require(s.step(2.5, in).motors, "motors after confirmation");
    in.stable_airborne = false;
    require(s.step(2.6, in).motors, "takeover stability is not an in-flight motor gate");
    in.feedback_ok = false;
    auto fault = s.step(2.7, in);
    require(fault.request_hold && !fault.heartbeat && !fault.motors, "feedback failure exits explicitly");
    in.feedback_ok = true;
    require(!s.step(3, in).motors && s.stage() == Stage::Stopped, "failure latches");
    OffboardSession existing;
    require(!existing.step(0, in).motors && existing.stage() == Stage::Stopped, "never steal existing Offboard");
    OffboardSession timeout;
    in.offboard = false;
    in.stable_airborne = true;
    timeout.step(0, in); timeout.step(1.3, in);
    require(!timeout.step(4.4, in).heartbeat && timeout.stage() == Stage::Stopped, "mode request timeout");
    OffboardSession manual;
    manual.step(0, in); manual.step(1.3, in);
    in.offboard = true; manual.step(1.4, in);
    in.offboard = false;
    require(!manual.step(1.5, in).request_hold && manual.stage() == Stage::Stopped, "manual exit respected");
    OffboardSession conflict;
    in.exclusive = false;
    require(!conflict.step(0, in).heartbeat && conflict.stage() == Stage::Stopped, "publisher conflict");
    require(px4_topic<Unversioned>("/test") == "/test", "old message topic");
    require(px4_topic<Versioned>("/test") == "/test_v5", "versioned message topic");
}
}
int main()
{
    try
    {
        allocation_tests(); reference_tests(); session_tests();
        std::cout << "PASS: 10000 allocation cases, yaw regression, motor mapping, trajectory derivatives, Offboard lifecycle, topic versions\n";
        return 0;
    }
    catch (const std::exception &e) { std::cerr << "FAIL: " << e.what() << '\n'; return 1; }
}
