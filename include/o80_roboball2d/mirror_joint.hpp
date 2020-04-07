#pragma once

#include "shared_memory/serializer.hpp"
#include "shared_memory/shared_memory.hpp"

#include "o80/interpolation.hpp"

#include "roboball2d_interface/item.hpp"

namespace o80_roboball2d
{
class MirrorJoint
{
public:
    MirrorJoint();
    MirrorJoint(double position, double velocity);

    void set(double position, double velocity);

    std::array<double, 2> get() const;

    void set_velocity(double value);
    void set_position(double value);

    double get_position() const;
    double get_velocity() const;

    std::string to_string() const;

    bool finished(const o80::TimePoint &start,
                  const o80::TimePoint &now,
                  const long int duration_us) const;

    bool finished(const o80::TimePoint &start,
                  const o80::TimePoint &now,
                  const MirrorJoint &start_state,
                  const MirrorJoint &current_state,
                  const MirrorJoint &previous_desired_state,
                  const MirrorJoint &target_state,
                  const o80::Speed &speed) const;

    MirrorJoint intermediate_state(const o80::TimePoint &start,
                                   const o80::TimePoint &now,
                                   const MirrorJoint &start_state,
                                   const MirrorJoint &current_state,
                                   const MirrorJoint &previous_desired_state,
                                   const MirrorJoint &target_state,
                                   const o80::Speed &speed) const;

    MirrorJoint intermediate_state(const o80::TimePoint &start,
                                   const o80::TimePoint &now,
                                   const MirrorJoint &start_state,
                                   const MirrorJoint &current_state,
                                   const MirrorJoint &previously_desired_state,
                                   const MirrorJoint &target_state,
                                   const o80::Duration_us &duration) const;

    MirrorJoint intermediate_state(long int iteration_start,
                                   long int iteration_now,
                                   const MirrorJoint &start_state,
                                   const MirrorJoint &current_state,
                                   const MirrorJoint &previous_desired_state,
                                   const MirrorJoint &target_state,
                                   const o80::Iteration &iteration) const;

    template <class Archive>
    void serialize(Archive &archive)
    {
        archive(position_, velocity_);
    }

private:
    friend shared_memory::private_serialization;
    double velocity_;
    double position_;
};
}
