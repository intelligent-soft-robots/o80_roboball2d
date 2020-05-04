#include <limits>

#include "real_time_tools/timer.hpp"

#include "robot_interfaces/robot.hpp"

#include "o80_roboball2d/ball_gun_action.hpp"
#include "o80_roboball2d/driver.hpp"
#include "o80_roboball2d/torques_action.hpp"
#include "o80_roboball2d/world_state.hpp"

using namespace o80_roboball2d;

#define NB_ROBOTS 1
#define NB_BALLS 1

int main()
{
    
    // should be the same value as the one set in
    // /python/o80_roboball2d/configuration.py
    std::string interface_id_real_robot = "real-robot";
    std::string interface_id_ball_gun = "real-ball-gun";

    typedef Driver<TorquesAction,NB_ROBOTS,NB_BALLS> DriverTorques;
    typedef Driver<BallGunAction,NB_ROBOTS,NB_BALLS> DriverBallGun;
    
    typedef robot_interfaces::Robot< TorquesAction,
				     WorldState<NB_ROBOTS,NB_BALLS>,
				     DriverTorques > 
        Robot;
    Robot robot(std::numeric_limits<double>::infinity(),
		std::numeric_limits<double>::infinity(),
                interface_id_real_robot);

    typedef robot_interfaces::Robot< BallGunAction,
				     WorldState<NB_ROBOTS,NB_BALLS>,
				     DriverBallGun >
        BallGun;

    BallGun ball_gun(std::numeric_limits<double>::infinity(),
		    std::numeric_limits<double>::infinity(),
		    interface_id_ball_gun);
    
    robot.initialize();
    ball_gun.initialize();

    double start_time = real_time_tools::Timer::get_current_time_sec();
    double current_time = start_time;

    // we will shoot a ball every 3 secs
    double ball_gun_time = start_time + 3.0;

    // shooting
    ball_gun.append_desired_action(BallGunAction(true));

    while (current_time - start_time < 10.0)
    {

        if (current_time > ball_gun_time)
        {
            ball_gun.append_desired_action(BallGunAction(true));
            ball_gun_time = current_time + 3.0;
        }

        current_time = real_time_tools::Timer::get_current_time_sec();

        double torque1 = 0.5 * cos(current_time);
        double torque2 = 0.5 * sin(current_time);
        double torque3 = torque1;

        bool relative_torques = true;

        TorquesAction action(torque1, torque2, torque3, relative_torques);

        robot_interfaces::TimeIndex index = robot.append_desired_action(action);
        WorldState<NB_ROBOTS,NB_BALLS> world_state = robot.get_observation(index);

        std::cout << current_time - start_time << "\t|\t"
                  << world_state.robot.joints[0].angle << "\t"
                  << world_state.robot.joints[1].angle << "\t"
                  << world_state.robot.joints[2].angle << "\n";

        real_time_tools::Timer::sleep_ms(10);
    }
}
