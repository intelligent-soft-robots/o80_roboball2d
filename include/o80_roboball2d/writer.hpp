#pragma once
#include "o80_roboball2d/world_state.hpp"
#include "shared_memory/shared_memory.hpp"

namespace o80_roboball2d
{
    template <class Action,int NB_ROBOTS=0, int NB_BALLS=0>
class Writer
{
public:
    Writer(std::string interface_id);
    static void clear(std::string interface_id);
    void write_world_state(const WorldState<NB_ROBOTS,NB_BALLS>& world_state) const;
    void write_action(const Action& action) const;

private:
    std::string interface_id_;
};

#include "writer.hxx"
}
