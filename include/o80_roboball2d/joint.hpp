#pragma once

#include "shared_memory/serializer.hpp"
#include "shared_memory/shared_memory.hpp"

#include "o80/interpolation.hpp"

#include "roboball2d_interface/item.hpp"

namespace o80_roboball2d
{
class Joint
{
public:
    Joint();
    Joint(double torque);

    Joint(double position, double velocity, double torque);

    void set(double torque);
    std::array<double, 3> get() const;

    void set_torque(double value);
    void set_velocity(double value);
    void set_position(double value);

    double get_position() const;
    double get_velocity() const;
    double get_torque() const;

    std::string to_string() const;

    bool finished(const o80::TimePoint &start,
                  const o80::TimePoint &now,
                  const long int duration_us) const;

    bool finished(const o80::TimePoint &start,
                  const o80::TimePoint &now,
                  const Joint &start_state,
                  const Joint &current_state,
                  const Joint &previous_desired_state,
                  const Joint &target_state,
                  const o80::Speed &speed) const;

    Joint intermediate_state(const o80::TimePoint &start,
                             const o80::TimePoint &now,
                             const Joint &start_state,
                             const Joint &current_state,
                             const Joint &previous_desired_state,
                             const Joint &target_state,
                             const o80::Speed &speed) const;

    Joint intermediate_state(const o80::TimePoint &start,
                             const o80::TimePoint &now,
                             const Joint &start_state,
                             const Joint &current_state,
                             const Joint &previously_desired_state,
                             const Joint &target_state,
                             const o80::Duration_us &duration) const;

    Joint intermediate_state(long int iteration_start,
                             long int iteration_now,
                             const Joint &start_state,
                             const Joint &current_state,
                             const Joint &previous_desired_state,
                             const Joint &target_state,
                             const o80::Iteration &iteration) const;

    template <class Archive>
    void serialize(Archive &archive)
    {
        archive(torque_, velocity_, position_);
    }

private:
    friend shared_memory::private_serialization;
    double torque_;
    double velocity_;
    double position_;
};
}
