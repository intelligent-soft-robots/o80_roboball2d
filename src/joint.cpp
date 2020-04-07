#include "o80_roboball2d/joint.hpp"

namespace o80_roboball2d
{
Joint::Joint() : position_(0.0), velocity_(0.0), torque_(0.0)
{
}

Joint::Joint(double torque) : position_(0.0), velocity_(0.0), torque_(torque)
{
}

Joint::Joint(double position, double velocity, double torque)
    : position_(position), velocity_(velocity), torque_(torque)
{
}

void Joint::set(double torque)
{
    torque_ = torque;
}

std::string Joint::to_string() const
{
    return std::to_string(position_) + std::string(" | ") +
           std::to_string(velocity_) + std::string(" | ") +
           std::to_string(torque_);
}

void Joint::set_torque(double value)
{
    torque_ = value;
}

void Joint::set_velocity(double value)
{
    velocity_ = value;
}

void Joint::set_position(double value)
{
    position_ = value;
}

std::array<double, 3> Joint::get() const
{
    return std::array<double, 3>{{position_, velocity_, torque_}};
}

double Joint::get_position() const
{
    return position_;
}

double Joint::get_velocity() const
{
    return velocity_;
}

double Joint::get_torque() const
{
    return torque_;
}

bool Joint::finished(const o80::TimePoint &start,
                     const o80::TimePoint &now,
                     const long int duration_us) const
{
    return o80::finished<double>(start, now, duration_us);
}

bool Joint::finished(const o80::TimePoint &start,
                     const o80::TimePoint &now,
                     const Joint &start_state,
                     const Joint &current_state,
                     const Joint &previous_desired_state,
                     const Joint &target_state,
                     const o80::Speed &speed) const
{
    return o80::finished(start,
                         now,
                         start_state.torque_,
                         current_state.torque_,
                         previous_desired_state.torque_,
                         speed);
}

Joint Joint::intermediate_state(const o80::TimePoint &start,
                                const o80::TimePoint &now,
                                const Joint &start_state,
                                const Joint &current_state,
                                const Joint &previous_desired_state,
                                const Joint &target_state,
                                const o80::Speed &speed) const
{
    double desired_torque =
        o80::intermediate_state<double>(start,
                                        now,
                                        start_state.torque_,
                                        previous_desired_state.torque_,
                                        target_state.torque_,
                                        speed);
    return Joint(current_state.get_position(),
                 current_state.get_velocity(),
                 desired_torque);
}

Joint Joint::intermediate_state(const o80::TimePoint &start,
                                const o80::TimePoint &now,
                                const Joint &start_state,
                                const Joint &current_state,
                                const Joint &previously_desired_state,
                                const Joint &target_state,
                                const o80::Duration_us &duration) const
{
    double desired_torque =
        o80::intermediate_state(start,
                                now,
                                start_state.torque_,
                                previously_desired_state.torque_,
                                target_state.torque_,
                                duration);
    return Joint(current_state.get_position(),
                 current_state.get_velocity(),
                 desired_torque);
}

Joint Joint::intermediate_state(long int iteration_start,
                                long int iteration_now,
                                const Joint &start_state,
                                const Joint &current_state,
                                const Joint &previous_desired_state,
                                const Joint &target_state,
                                const o80::Iteration &iteration) const
{
    double desired_torque =
        o80::intermediate_state(iteration_start,
                                iteration_now,
                                start_state.torque_,
                                previous_desired_state.torque_,
                                target_state.torque_,
                                iteration);
    return Joint(current_state.get_position(),
                 current_state.get_velocity(),
                 desired_torque);
}
}
