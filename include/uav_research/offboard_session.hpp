#pragma once
namespace uav_research
{
struct OffboardInputs
{
    bool enabled = false, feedback_ok = false, armed = false, failsafe = false;
    bool offboard = false, stable_airborne = false, exclusive = true;
};
struct OffboardActions
{
    bool heartbeat = false, motors = false, request_offboard = false, request_hold = false;
};
// Pure state machine: tested without ROS. A stopped session cannot re-enter.
class OffboardSession
{
public:
    enum class Stage { Waiting, Priming, Requested, Active, Stopped };
    Stage stage() const { return stage_; }
    const char *reason() const { return reason_; }
    OffboardActions step(double now, const OffboardInputs &in)
    {
        OffboardActions out;
        if (!in.enabled || stage_ == Stage::Stopped) return out;
        const bool owns = stage_ == Stage::Requested || stage_ == Stage::Active;
        if (!in.exclusive)
        {
            stop("Another Offboard/motor publisher detected");
            return out;
        }
        if (!owns && in.offboard)
        {
            stop("Already in Offboard before our request; select Hold and restart");
            return out;
        }
        if (stage_ == Stage::Active && !in.offboard)
        {
            stop("PX4 exited Offboard; restart required");
            return out;
        }
        if (!in.feedback_ok || !in.armed || in.failsafe)
        {
            if (owns)
            {
                out.request_hold = in.offboard && in.armed && !in.failsafe;
                stop("Feedback invalid/stale, disarmed, or PX4 failsafe");
            }
            else stage_ = Stage::Waiting;
            return out;
        }
        if (stage_ == Stage::Requested)
        {
            if (in.offboard) { stage_ = Stage::Active; reason_ = "PX4 confirmed Offboard"; }
            else if (now-since_ > 3.0)
            {
                stop("Offboard request timed out; check ACK and topic versions");
                return out;
            }
            out.heartbeat = true;
            out.motors = stage_ == Stage::Active;
            return out;
        }
        if (stage_ == Stage::Active)
        {
            out.heartbeat = out.motors = true;
            return out;
        }
        if (!in.stable_airborne)
        {
            stage_ = Stage::Waiting;
            return out;
        }
        if (stage_ == Stage::Waiting)
        {
            since_ = now;
            stage_ = Stage::Priming;
            reason_ = "Prestreaming heartbeat while stable in Hold/Position";
        }
        out.heartbeat = true;
        if (now-since_ >= 1.2)
        {
            since_ = now;
            stage_ = Stage::Requested;
            reason_ = "Waiting for PX4 to confirm Offboard";
            out.request_offboard = true;
        }
        return out;
    }
private:
    void stop(const char *reason) { stage_ = Stage::Stopped; reason_ = reason; }
    Stage stage_ = Stage::Waiting;
    double since_ = 0;
    const char *reason_ = "Waiting for stable airborne feedback";
};
} // namespace uav_research
