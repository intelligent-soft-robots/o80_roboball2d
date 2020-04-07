#pragma once

#include "o80/bool_state.hpp"
#include "o80/standalone.hpp"

#include "roboball2d_interface/ball_gun_action.hpp"
#include "roboball2d_interface/driver.hpp"
#include "roboball2d_interface/world_state.hpp"

namespace o80_roboball2d
{
    template <int QUEUE_SIZE,int NB_ROBOTS,int NB_BALLS>
class StandaloneBallGun
    : public o80::Standalone<QUEUE_SIZE,
                             1,
                             roboball2d_interface::BallGunAction,
                             roboball2d_interface::WorldState<NB_ROBOTS,NB_BALLS>,
                             o80::BoolState,
                             roboball2d_interface::WorldState<NB_ROBOTS,NB_BALLS>>
{
public:
    StandaloneBallGun(std::shared_ptr<roboball2d_interface::Driver<
		      roboball2d_interface::BallGunAction,NB_ROBOTS,NB_BALLS>> driver_ptr,
                      double frequency,
                      std::string segment_id)
        : o80::Standalone<QUEUE_SIZE,
                          1,
                          roboball2d_interface::BallGunAction,
                          roboball2d_interface::WorldState<NB_ROBOTS,NB_BALLS>,
                          o80::BoolState,
                          roboball2d_interface::WorldState<NB_ROBOTS,NB_BALLS>>(
              driver_ptr,
              frequency,
              segment_id)
    {
    }

    roboball2d_interface::BallGunAction convert(
        const o80::States<1, o80::BoolState> &bool_state)
    {
        roboball2d_interface::BallGunAction action;
        if (bool_state.get(0).get())
        {
            action.set_ball_gun(true);
        }
        else
        {
            action.set_ball_gun(false);
        }
        return action;
    }

    o80::States<1, o80::BoolState> convert(
        const roboball2d_interface::WorldState<NB_ROBOTS,NB_BALLS> &world_state)
    {
        o80::States<1, o80::BoolState> states;
        return states;
    }

    void enrich_extended_state(roboball2d_interface::WorldState<NB_ROBOTS,NB_BALLS>& to,
                               const roboball2d_interface::WorldState<NB_ROBOTS,NB_BALLS>& from)
    {
        to = from;
    }
};
}
