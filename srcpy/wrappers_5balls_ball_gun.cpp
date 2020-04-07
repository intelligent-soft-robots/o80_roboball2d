#include "o80/pybind_helper.hpp"
#include "o80_roboball2d/standalone_ball_gun.hpp"

#define QUEUE 5000
#define NB_ROBOTS 1
#define NB_BALLS 5
#define TYPE 6

using namespace o80_roboball2d;
using namespace roboball2d_interface;


PYBIND11_MODULE(o80_roboball2d_5balls_ball_gun, m)
{
    o80::create_python_bindings<Driver<BallGunAction,NB_ROBOTS,NB_BALLS,TYPE>,
				StandaloneBallGun<QUEUE,NB_ROBOTS,NB_BALLS,TYPE>,
			    std::string>(m, false, false, true);
}
