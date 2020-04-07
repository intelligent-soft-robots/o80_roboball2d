#include "o80/pybind_helper.hpp"
#include "o80_roboball2d/standalone_torques.hpp"

#define QUEUE 5000
#define NB_ROBOTS 1
#define NB_BALLS 1

using namespace o80_roboball2d;
using namespace roboball2d_interface;

PYBIND11_MODULE(o80_roboball2d_torques, m)
{
    o80::create_python_bindings<Driver<TorquesAction,NB_ROBOTS,NB_BALLS>,
                                StandaloneTorques<QUEUE,NB_ROBOTS,NB_BALLS>,
                                std::string>(m, false, false, false, true);

}
