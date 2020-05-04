

template <class Action,int NB_ROBOTS, int NB_BALLS>
Driver<Action,NB_ROBOTS,NB_BALLS>::Driver(std::string interface_id)
    : interface_id_(interface_id), writer_(interface_id), reader_(interface_id)
{
}

template <class Action,int NB_ROBOTS, int NB_BALLS>
void Driver<Action,NB_ROBOTS,NB_BALLS>::initialize()
{
}

template <class Action,int NB_ROBOTS, int NB_BALLS>
Action Driver<Action,NB_ROBOTS,NB_BALLS>::apply_action(const Action &desired_action)
{
    std::lock_guard<std::mutex> guard(mutex_);
    if (desired_action.is_valid())
	{
	    writer_.write_action(desired_action);
	}
    return desired_action;
}

template <class Action,int NB_ROBOTS, int NB_BALLS>
WorldState<NB_ROBOTS,NB_BALLS> Driver<Action,NB_ROBOTS,NB_BALLS>::get_latest_observation()
{
    WorldState<NB_ROBOTS,NB_BALLS> ws = reader_.read_world_state();
    return ws;
}

template <class Action,int NB_ROBOTS, int NB_BALLS>
void Driver<Action,NB_ROBOTS,NB_BALLS>::shutdown()
{
}

template <class Action,int NB_ROBOTS, int NB_BALLS>
std::string Driver<Action,NB_ROBOTS,NB_BALLS>::get_error()
{
    return "";
}
