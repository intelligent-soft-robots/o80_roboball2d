#include "o80/pybind_helper.hpp"
#include "o80_roboball2d/joint.hpp"
#include "o80_roboball2d/mirror_joint.hpp"

using namespace o80_roboball2d;
using namespace roboball2d_interface;

PYBIND11_MODULE(o80_roboball2d, m)
{
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

    pybind11::class_<MirrorJoint>(m, "MirrorJoint")
        .def(pybind11::init<>())
        .def("set", &MirrorJoint::set)
        .def("get", &MirrorJoint::get)
        .def("to_string", &MirrorJoint::to_string)
        .def("set_velocity", &MirrorJoint::set_velocity)
        .def("set_position", &MirrorJoint::set_position)
        .def("get_position", &MirrorJoint::get_position)
        .def("get_velocity", &MirrorJoint::get_velocity);

    typedef o80::States<3, MirrorJoint> MirrorJointStates;
    pybind11::class_<MirrorJointStates>(m, "MirrorJointStates")
	.def(pybind11::init<>())
	.def("set", &MirrorJointStates::set)
	.def("get", &MirrorJointStates::get)
	.def_readwrite("values", &MirrorJointStates::values);

    typedef o80::States<3, Joint> JointStates;
    pybind11::class_<JointStates>(m, "JointStates")
	.def(pybind11::init<>())
	.def("set", &JointStates::set)
	.def("get", &JointStates::get)
	.def_readwrite("values", &JointStates::values);

    
    
}
