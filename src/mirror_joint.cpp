#include "o80_roboball2d/mirror_joint.hpp"

namespace o80_roboball2d
{
MirrorJoint::MirrorJoint() : position_(0.0), velocity_(0.0)
{
}

MirrorJoint::MirrorJoint(double position, double velocity)
    : position_(position), velocity_(velocity)
{
}

void MirrorJoint::set(double position, double velocity)
{
    position_ = position;
    velocity_ = velocity;
}

std::string MirrorJoint::to_string() const
{
    return std::to_string(position_) + std::string(" | ") +
           std::to_string(velocity_) + std::string(" | ");
}

void MirrorJoint::set_velocity(double value)
{
    velocity_ = value;
}

void MirrorJoint::set_position(double value)
{
    position_ = value;
}

std::array<double, 2> MirrorJoint::get() const
{
    return std::array<double, 2>{{position_, velocity_}};
}

double MirrorJoint::get_position() const
{
    return position_;
}

double MirrorJoint::get_velocity() const
{
    return velocity_;
}

bool MirrorJoint::finished(const o80::TimePoint &start,
                           const o80::TimePoint &now,
                           const long int duration_us) const
{
    throw std::runtime_error("MirrorJoint: only direct commands supported");
}

bool MirrorJoint::finished(const o80::TimePoint &start,
                           const o80::TimePoint &now,
                           const MirrorJoint &start_state,
                           const MirrorJoint &current_state,
                           const MirrorJoint &previous_desired_state,
                           const MirrorJoint &target_state,
                           const o80::Speed &speed) const
{
    throw std::runtime_error("MirrorJoint: only direct commands supported");
}

MirrorJoint MirrorJoint::intermediate_state(
    const o80::TimePoint &start,
    const o80::TimePoint &now,
    const MirrorJoint &start_state,
    const MirrorJoint &current_state,
    const MirrorJoint &previous_desired_state,
    const MirrorJoint &target_state,
    const o80::Speed &speed) const
{
    throw std::runtime_error("MirrorJoint: only direct commands supported");
}

MirrorJoint MirrorJoint::intermediate_state(
    const o80::TimePoint &start,
    const o80::TimePoint &now,
    const MirrorJoint &start_state,
    const MirrorJoint &current_state,
    const MirrorJoint &previously_desired_state,
    const MirrorJoint &target_state,
    const o80::Duration_us &duration) const
{
    throw std::runtime_error("MirrorJoint: only direct commands supported");
}

MirrorJoint MirrorJoint::intermediate_state(
    long int iteration_start,
    long int iteration_now,
    const MirrorJoint &start_state,
    const MirrorJoint &current_state,
    const MirrorJoint &previous_desired_state,
    const MirrorJoint &target_state,
    const o80::Iteration &iteration) const
{
    throw std::runtime_error("MirrorJoint: only direct commands supported");
}
}
