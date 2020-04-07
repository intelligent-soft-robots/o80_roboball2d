#include "o80/pybind_helper.hpp"
#include "o80_roboball2d/standalone_mirroring.hpp"

#define QUEUE 5000
#define NB_ROBOTS 1
#define NB_BALLS 5
#define TYPE 5

using namespace o80_roboball2d;
using namespace roboball2d_interface;

PYBIND11_MODULE(o80_roboball2d_5balls_mirroring, m)
{
    o80::create_python_bindings<Driver<MirrorAction,NB_ROBOTS,NB_BALLS,TYPE>,
                                StandaloneMirroring<QUEUE,NB_ROBOTS,NB_BALLS,TYPE>,
                                std::string>(m, false, false, true);

    pybind11::class_<MirrorJoint>(m, "MirrorJoint")
        .def(pybind11::init<>())
        .def("set", &MirrorJoint::set)
        .def("get", &MirrorJoint::get)
        .def("to_string", &MirrorJoint::to_string)
        .def("set_velocity", &MirrorJoint::set_velocity)
        .def("set_position", &MirrorJoint::set_position)
        .def("get_position", &MirrorJoint::get_position)
        .def("get_velocity", &MirrorJoint::get_velocity);
}
