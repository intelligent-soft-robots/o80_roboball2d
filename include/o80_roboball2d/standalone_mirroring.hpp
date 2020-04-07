#pragma once

#include "o80/standalone.hpp"

#include "roboball2d_interface/driver.hpp"
#include "roboball2d_interface/mirror_action.hpp"
#include "roboball2d_interface/world_state.hpp"

#include "o80_roboball2d/mirror_joint.hpp"

namespace o80_roboball2d
{
    template <int QUEUE_SIZE,int NB_ROBOTS,int NB_BALLS,int TYPE>
class StandaloneMirroring
    : public o80::Standalone< QUEUE_SIZE,
			      3,
			      roboball2d_interface::MirrorAction,
			      roboball2d_interface::WorldState<NB_ROBOTS,NB_BALLS,TYPE>,
			      MirrorJoint,
			      roboball2d_interface::WorldState<NB_ROBOTS,NB_BALLS,TYPE> >
{
public:
    StandaloneMirroring(std::shared_ptr<roboball2d_interface::Driver<
			roboball2d_interface::MirrorAction,NB_ROBOTS,NB_BALLS,TYPE>> driver_ptr,
                        double frequency,
                        std::string segment_id)
        : o80::Standalone<QUEUE_SIZE,
                          3,
                          roboball2d_interface::MirrorAction,
                          roboball2d_interface::WorldState<NB_ROBOTS,NB_BALLS,TYPE>,
                          MirrorJoint,
                          roboball2d_interface::WorldState<NB_ROBOTS,NB_BALLS,TYPE>>(
              driver_ptr,
              frequency,
              segment_id)
    {
    }

    roboball2d_interface::MirrorAction convert(
        const o80::States<3, MirrorJoint> &joints)
    {
        roboball2d_interface::MirrorAction action;
        action.set_mirroring_information(
            joints.get(0).get(), joints.get(1).get(), joints.get(2).get());

        return action;
    }

    o80::States<3, MirrorJoint> convert(
					const roboball2d_interface::WorldState<NB_ROBOTS,NB_BALLS,TYPE> &world_state)
    {
        o80::States<3, MirrorJoint> states;
        return states;
    }

    void enrich_extended_state(roboball2d_interface::WorldState<NB_ROBOTS,NB_BALLS,TYPE>& to,
                               const roboball2d_interface::WorldState<NB_ROBOTS,NB_BALLS,TYPE>& from)
    {
        to = from;
    }
};
}
