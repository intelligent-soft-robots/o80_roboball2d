#include "o80_roboball2d/ball_gun_driver.hpp"


namespace o80_roboball2d
{


BallGunDriver::BallGunDriver(std::string interface_id)
    : interface_id_(interface_id), writer_(interface_id)
{
}

void BallGunDriver::initialize()
{
}

BallGunAction BallGunDriver::apply_action(const BallGunAction &desired_action)
{
    std::lock_guard<std::mutex> guard(mutex_);
    if (desired_action.is_valid())
	{
	    writer_.write_action(desired_action);
	}
    return desired_action;
}

    o80::VoidObservation BallGunDriver::get_latest_observation()
{
    return o80::VoidObservation();
}

void BallGunDriver::shutdown()
{
}

std::string BallGunDriver::get_error()
{
    return "";
}
    
}
