#pragma once

#include <mutex>

#include "robot_interfaces/robot_driver.hpp"

#include "o80_roboball2d/ball_gun_action.hpp"
#include "o80_roboball2d/mirror_action.hpp"
#include "o80_roboball2d/reader.hpp"
#include "o80_roboball2d/torques_action.hpp"
#include "o80_roboball2d/world_state.hpp"
#include "o80_roboball2d/writer.hpp"

namespace o80_roboball2d
{
    template <class Action,int NB_ROBOTS,int NB_BALLS>
    class Driver : public robot_interfaces::RobotDriver< Action,
							 WorldState<NB_ROBOTS,NB_BALLS > >

{
public:
    Driver(std::string interface_id);
    void initialize();
    Action apply_action(const Action &desired_action);
    WorldState<NB_ROBOTS,NB_BALLS> get_latest_observation();
    void shutdown();
    std::string get_error();

private:
    std::string interface_id_;
    Writer<Action,NB_ROBOTS,NB_BALLS> writer_;
    Reader<Action,NB_ROBOTS,NB_BALLS> reader_;
    bool should_shoot_;

private:
    static std::mutex mutex_;
};

    template <class Action,int NB_ROBOTS,int NB_BALLS>
    std::mutex Driver<Action,NB_ROBOTS,NB_BALLS>::mutex_;

#include "driver.hxx"
}
