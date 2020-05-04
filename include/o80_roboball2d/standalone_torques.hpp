#pragma once

#include "o80/standalone.hpp"

#include "o80_roboball2d/driver.hpp"
#include "o80_roboball2d/torques_action.hpp"
#include "o80_roboball2d/world_state.hpp"

#include "o80_roboball2d/joint.hpp"

namespace o80_roboball2d
{
    template <int QUEUE_SIZE,int NB_ROBOTS,int NB_BALLS>
class StandaloneTorques
    : public o80::Standalone<QUEUE_SIZE,
                             3,
                             o80_roboball2d::TorquesAction,
                             o80_roboball2d::WorldState<NB_ROBOTS,NB_BALLS>,
                             Joint,
                             o80_roboball2d::WorldState<NB_ROBOTS,NB_BALLS>>
{
public:
    StandaloneTorques(std::shared_ptr<o80_roboball2d::Driver<
		      o80_roboball2d::TorquesAction,NB_ROBOTS,NB_BALLS>> driver_ptr,
                      double frequency,
                      std::string segment_id)
        : o80::Standalone<QUEUE_SIZE,
                          3,
                          o80_roboball2d::TorquesAction,
                          o80_roboball2d::WorldState<NB_ROBOTS,NB_BALLS>,
                          Joint,
                          o80_roboball2d::WorldState<NB_ROBOTS,NB_BALLS>>(
              driver_ptr,
              frequency,
              segment_id)
    {
    }

    o80_roboball2d::TorquesAction convert(
        const o80::States<3, Joint> &joints)
    {
        o80_roboball2d::TorquesAction action;
        action.set_torques(joints.get(0).get_torque(),
                           joints.get(1).get_torque(),
                           joints.get(2).get_torque());
        return action;
    }

    o80::States<3, Joint> convert(
				  const o80_roboball2d::WorldState<NB_ROBOTS,NB_BALLS> &world_state)
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

    void enrich_extended_state(o80_roboball2d::WorldState<NB_ROBOTS,NB_BALLS>& to,
                               const o80_roboball2d::WorldState<NB_ROBOTS,NB_BALLS>& from)
    {
        to = from;
    }
};
}
