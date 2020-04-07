#pragma once

#include "o80/standalone.hpp"
#include "o80/void_state.hpp"

#include "roboball2d_interface/driver.hpp"
#include "roboball2d_interface/torques_action.hpp"
#include "roboball2d_interface/world_state.hpp"

namespace o80_roboball2d
{
    template <int QUEUE_SIZE,int NB_ROBOTS,int NB_BALLS>
class StandaloneVision
    : public o80::Standalone<QUEUE_SIZE,
                             1,
                             roboball2d_interface::TorquesAction,
                             roboball2d_interface::WorldState<NB_ROBOTS,NB_BALLS>,
                             o80::VoidState,
                             roboball2d_interface::WorldState<NB_ROBOTS,NB_BALLS>>
{
public:
    StandaloneVision(std::shared_ptr<roboball2d_interface::Driver<
		     roboball2d_interface::TorquesAction,NB_ROBOTS,NB_BALLS>> driver_ptr,
                     double frequency,
                     std::string segment_id)
        : o80::Standalone<QUEUE_SIZE,
                          1,
                          roboball2d_interface::TorquesAction,
                          roboball2d_interface::WorldState<NB_ROBOTS,NB_BALLS>,
                          o80::VoidState,
                          roboball2d_interface::WorldState<NB_ROBOTS,NB_BALLS>>(
              driver_ptr,
              frequency,
              segment_id)
    {
    }

    roboball2d_interface::TorquesAction convert(
        const o80::States<1, o80::VoidState> &void_state)
    {
        roboball2d_interface::TorquesAction action;
        return action;
    }

    o80::States<1, o80::VoidState> convert(
					   const roboball2d_interface::WorldState<NB_ROBOTS,NB_BALLS> &world_state)
    {
        o80::States<1, o80::VoidState> states;
        return states;
    }

    void enrich_extended_state(roboball2d_interface::WorldState<NB_ROBOTS,NB_BALLS>& to,
                               const roboball2d_interface::WorldState<NB_ROBOTS,NB_BALLS>& from)
    {
        to = from;
    }
};
}
