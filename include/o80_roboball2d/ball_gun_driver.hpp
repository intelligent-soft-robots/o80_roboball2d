#pragma once

#include <mutex>
#include "o80/void_observation.hpp"
#include "robot_interfaces/robot_driver.hpp"
#include "o80_roboball2d/ball_gun_action.hpp"
#include "o80_roboball2d/writer.hpp"

namespace o80_roboball2d
{

    class BallGunDriver
	: public robot_interfaces::RobotDriver<BallGunAction,
					       o80::VoidObservation>
    {
    public:
	BallGunDriver(std::string interface_id);
	void initialize();
	BallGunAction apply_action(const BallGunAction &desired_action);
	o80::VoidObservation get_latest_observation();
	void shutdown();
	std::string get_error();

    private:
	std::string interface_id_;
	Writer<BallGunAction> writer_;
	bool should_shoot_;

    private:
	static std::mutex mutex_;
    };

    std::mutex BallGunDriver::mutex_;

}
