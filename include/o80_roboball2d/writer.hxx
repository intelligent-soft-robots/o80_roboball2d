
template <class Action,int NB_ROBOTS, int NB_BALLS>
Writer<Action,NB_ROBOTS,NB_BALLS>::Writer(std::string interface_id) : interface_id_(interface_id)
{
}

template <class Action,int NB_ROBOTS, int NB_BALLS>
void Writer<Action,NB_ROBOTS,NB_BALLS>::clear(std::string interface_id)
{
    shared_memory::clear_shared_memory(interface_id);
}

    template <class Action,int NB_ROBOTS, int NB_BALLS>
    void Writer<Action,NB_ROBOTS,NB_BALLS>::write_world_state(const WorldState<NB_ROBOTS,NB_BALLS>& world_state) const
{
    shared_memory::serialize(interface_id_, "world_state", world_state);
}


    template <class Action,int NB_ROBOTS, int NB_BALLS>
void Writer<Action,NB_ROBOTS,NB_BALLS>::write_action(const Action& action) const
{
    shared_memory::serialize(interface_id_, "action", action);
}



