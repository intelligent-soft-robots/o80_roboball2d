#pragma once

#include "o80/standalone.hpp"
#include "o80/void_state.hpp"

#include "o80_roboball2d/driver.hpp"
#include "o80_roboball2d/torques_action.hpp"
#include "o80_roboball2d/world_state.hpp"

namespace o80_roboball2d
{
    template <int QUEUE_SIZE,int NB_ROBOTS,int NB_BALLS>
class StandaloneVision
    : public o80::Standalone<QUEUE_SIZE,
                             1,
                             o80_roboball2d::TorquesAction,
                             o80_roboball2d::WorldState<NB_ROBOTS,NB_BALLS>,
                             o80::VoidState,
                             o80_roboball2d::WorldState<NB_ROBOTS,NB_BALLS>>
{
public:
    StandaloneVision(std::shared_ptr<o80_roboball2d::Driver<
		     o80_roboball2d::TorquesAction,NB_ROBOTS,NB_BALLS>> driver_ptr,
                     double frequency,
                     std::string segment_id)
        : o80::Standalone<QUEUE_SIZE,
                          1,
                          o80_roboball2d::TorquesAction,
                          o80_roboball2d::WorldState<NB_ROBOTS,NB_BALLS>,
                          o80::VoidState,
                          o80_roboball2d::WorldState<NB_ROBOTS,NB_BALLS>>(
              driver_ptr,
              frequency,
              segment_id)
    {
    }

    o80_roboball2d::TorquesAction convert(
        const o80::States<1, o80::VoidState> &void_state)
    {
        o80_roboball2d::TorquesAction action;
        return action;
    }

    o80::States<1, o80::VoidState> convert(
					   const o80_roboball2d::WorldState<NB_ROBOTS,NB_BALLS> &world_state)
    {
        o80::States<1, o80::VoidState> states;
        return states;
    }

    void enrich_extended_state(o80_roboball2d::WorldState<NB_ROBOTS,NB_BALLS>& to,
                               const o80_roboball2d::WorldState<NB_ROBOTS,NB_BALLS>& from)
    {
        to = from;
    }
};
}
