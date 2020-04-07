#include "o80/pybind_helper.hpp"
#include "o80_roboball2d/standalone_torques.hpp"

#define QUEUE 5000
#define NB_ROBOTS 1
#define NB_BALLS 1
#define TYPE = 0

using namespace o80_roboball2d;
using namespace roboball2d_interface;

PYBIND11_MODULE(o80_roboball2d_torques, m)
{
    o80::create_python_bindings<Driver<TorquesAction,NB_ROBOTS,NB_BALLS,TYPE>,
                                StandaloneTorques<QUEUE,NB_ROBOTS,NB_BALLS,TYPE>,
                                std::string>(m, false, false, true);

    pybind11::class_<Joint>(m, "Joint")
        .def(pybind11::init<>())
        .def("set", &Joint::set)
        .def("get", &Joint::get)
        .def("to_string", &Joint::to_string)
        .def("set_torque", &Joint::set_torque)
        .def("set_velocity", &Joint::set_velocity)
        .def("set_position", &Joint::set_position)
        .def("get_position", &Joint::get_position)
        .def("get_velocity", &Joint::get_velocity)
        .def("get_torque", &Joint::get_torque);
}
