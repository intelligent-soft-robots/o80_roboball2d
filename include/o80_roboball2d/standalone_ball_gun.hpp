#pragma once

#include "o80/bool_state.hpp"
#include "o80/void_observation.hpp"
#include "o80/standalone.hpp"
#include "o80_roboball2d/ball_gun_action.hpp"
#include "o80_roboball2d/ball_gun_driver.hpp"

namespace o80_roboball2d
{
    template <int QUEUE_SIZE>
    class StandaloneBallGun
	: public o80::Standalone<QUEUE_SIZE,
				 1,
				 o80_roboball2d::BallGunAction,
				 o80::VoidObservation,
				 o80::BoolState,
				 o80::VoidObservation>
    {
    public:
	StandaloneBallGun(std::shared_ptr<o80_roboball2d::BallGunDriver> driver_ptr,
			  double frequency,
			  std::string segment_id)
	    : o80::Standalone<QUEUE_SIZE,
			      1,
			      o80_roboball2d::BallGunAction,
			      o80::VoidObservation,
			      o80::BoolState,
			      o80::VoidObservation>(driver_ptr,
								     frequency,
								     segment_id)
	    {
	    }

	o80_roboball2d::BallGunAction convert(
						    const o80::States<1, o80::BoolState> &bool_state)
	{
	    o80_roboball2d::BallGunAction action;
	    if (bool_state.get(0).get())
		{
		    action.set(true);
		}
	    else
		{
		    action.set(false);
		}
	    return action;
	}

	o80::States<1, o80::BoolState> convert(
					       const o80::VoidObservation &void_observation)
	{
	    o80::States<1, o80::BoolState> states;
	    return states;
	}

	void enrich_extended_state(o80::VoidObservation& to,
				   const o80::VoidObservation& from)
	{
	    to = from;
	}
    };
}
