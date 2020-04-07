#pragma once

#include "o80/standalone.hpp"

#include "o80_roboball2d/joint.hpp"
#include "o80_roboball2d/roboball2d_interface/action.hpp"
#include "o80_roboball2d/roboball2d_interface/driver.hpp"
#include "o80_roboball2d/roboball2d_interface/world_state.hpp"

namespace o80_roboball2d
{
template <int QUEUE_SIZE>
class StandaloneTorques
    : public o80::Standalone<QUEUE_SIZE,
                             3,
                             roboball2d_interface::Action,
                             roboball2d_interface::WorldState,
                             Joint,
                             roboball2d_interface::WorldState>
{
public:
    StandaloneTorques(std::shared_ptr<roboball2d_interface::Driver> driver_ptr,
                      double max_action_duration_s,
                      double max_inter_action_duration_s,
                      double frequency,
                      std::string segment_id)
        : o80::Standalone<QUEUE_SIZE,
                          3,
                          roboball2d_interface::Action,
                          roboball2d_interface::WorldState,
                          Joint,
                          roboball2d_interface::WorldState>(
              driver_ptr,
              max_action_duration_s,
              max_inter_action_duration_s,
              frequency,
              segment_id)
    {
    }

    roboball2d_interface::Action convert(const o80::States<3, Joint> &joints)
    {
        roboball2d_interface::Action action;
        action.set_torques(joints.get(0).get_torque(),
                           joints.get(1).get_torque(),
                           joints.get(2).get_torque());
        action.valid = true;
        return action;
    }

    o80::States<3, Joint> convert(
        const roboball2d_interface::WorldState &world_state)
    {
        o80::States<3, Joint> states;
        for (uint dof = 0; dof < 3; dof++)
        {
            double torque = world_state.robot.joints[dof].torque;
            double angle = world_state.robot.joints[dof].angle;
            double angular_velocity =
                world_state.robot.joints[dof].angular_velocity;
            states.set(dof, Joint(angle, angular_velocity, torque));
        }
        return states;
    }

    void enrich_extended_state(roboball2d_interface::WorldState to,
                               const roboball2d_interface::WorldState from)
    {
        to = from;
    }
};
}
