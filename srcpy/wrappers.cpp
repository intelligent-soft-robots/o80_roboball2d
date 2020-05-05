#include "o80/pybind11_helper.hpp"
#include "o80/void_state.hpp"
#include "o80/bool_state.hpp"
#include "o80_roboball2d/joint.hpp"
#include "o80_roboball2d/mirror_joint.hpp"
#include "o80_roboball2d/world_state.hpp"

#define QUEUE 5000
#define NB_DOFS 3 // roboball2d has 3 dofs

using namespace o80_roboball2d;

template<class WSClass>
void add_world_state(pybind11::module &m,
		     std::string classname)
{
    pybind11::class_<WSClass>(m,classname.c_str())
	.def(pybind11::init<>())
	.def_readonly("id", &WSClass::id)
	.def_readonly("valid", &WSClass::valid)
	.def_readwrite("t", &WSClass::t)
	.def_readwrite("ball", &WSClass::ball)
	.def_readwrite("robot", &WSClass::robot)
	.def_readwrite("balls", &WSClass::balls)
	.def_readwrite("robots", &WSClass::robots)
	.def_readwrite("ball_hits_floor_position",
		       &WSClass::ball_hits_floor_position)
	.def_readwrite("ball_hits_floor", &WSClass::ball_hits_floor)
	.def_readwrite("ball_hits_racket", &WSClass::ball_hits_racket)
	.def_readwrite("balls_hits_floor_position",
		       &WSClass::balls_hits_floor_position)
	.def_readwrite("balls_hits_floor", &WSClass::balls_hits_floor)
	.def_readwrite("balls_hits_racket", &WSClass::balls_hits_racket)
	.def("console", &WSClass::console);
}


PYBIND11_MODULE(o80_roboball2d, m)
{

    pybind11::class_<o80_roboball2d::Item>(m,"Item")
	.def(pybind11::init<>())
	.def("console",&o80_roboball2d::Item::console)
	.def_readwrite("position",&o80_roboball2d::Item::position)
	.def_readwrite("angle",&o80_roboball2d::Item::angle)
	.def_readwrite("linear_velocity",&o80_roboball2d::Item::linear_velocity)
	.def_readwrite("angular_velocity",&o80_roboball2d::Item::angular_velocity)
	.def_readwrite("torque",&o80_roboball2d::Item::torque)
	.def_readwrite("desired_torque",&o80_roboball2d::Item::desired_torque);

    pybind11::class_<o80_roboball2d::Robot>(m,"Robot")
	.def(pybind11::init<>())
	.def("console",&o80_roboball2d::Robot::console)
	.def_readwrite("joints",&o80_roboball2d::Robot::joints)
	.def_readwrite("rods",&o80_roboball2d::Robot::rods)
	.def_readwrite("racket",&o80_roboball2d::Robot::racket);
    
    pybind11::class_<MirrorJoint>(m, "MirrorJoint")
	.def(pybind11::init<>())
	.def("set", &MirrorJoint::set)
	.def("get", &MirrorJoint::get)
	.def("to_string", &MirrorJoint::to_string)
	.def("set_velocity", &MirrorJoint::set_velocity)
	.def("set_position", &MirrorJoint::set_position)
	.def("get_position", &MirrorJoint::get_position)
	.def("get_velocity", &MirrorJoint::get_velocity);

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
    
    
    add_world_state<OneBallWorldState>(m,std::string("OneBallWorldState"));
    add_world_state<FiveBallsWorldState>(m,std::string("FiveBallsWorldState"));


    // for pseudo robot control
    
    o80::Pybind11Config config_real_robot;
    config_real_robot.extended_state = false; // VoidExtendedState, unused
    config_real_robot.state = false; // Joint, added above
    config_real_robot.prefix = "RealRobot";
    o80::create_core_python_bindings<QUEUE,
				     NB_DOFS,
				     Joint,
				     o80::VoidExtendedState>(m,config_real_robot);

    // for ball-gun control

    o80::Pybind11Config config_ball_gun;
    config_ball_gun.state = false; // o80::BoolState
    config_ball_gun.prefix = "BallGun";
    o80::create_core_python_bindings<QUEUE,
				     NB_DOFS,
				     o80::BoolState,
				     o80::VoidExtendedState>(m,config_ball_gun);


    // for mirroring robot
    
    o80::Pybind11Config config_mirroring;
    config_mirroring.state = false; // added above (MirrorJoint, done above)
    config_mirroring.extended_state = false; // generic VoidExtendedState (unused)
    config_mirroring.prefix = "Mirroring";
    o80::create_core_python_bindings<QUEUE,
				     3,
				     MirrorJoint,
				     o80::VoidExtendedState>(m,config_mirroring);

    
    // for exchanging 1 ball information

    o80::Pybind11Config config_ball;
    config_ball.state = false; // VoidState (unused)
    config_ball.extended_state = false; // Item (done above)
    config_ball.prefix = "Ball";
    o80::create_core_python_bindings<QUEUE,
				     1,
				     o80::VoidState,
				     Item>(m,config_ball);
    
    // for returning world state (five balls)
    
    o80::Pybind11Config config_five_balls_world_state;
    config_five_balls_world_state.state = false; // VoidState (unused)
    config_five_balls_world_state.states = false; // VoidState (unused)
    config_five_balls_world_state.extended_state = false; // FiveBallsWorldsState (done above)
    config_five_balls_world_state.prefix = "FiveBallsWorldState";
    o80::create_core_python_bindings<QUEUE,
				     1,
				     o80::VoidState,
				     FiveBallsWorldState>(m,config_five_balls_world_state);
    
    
    
}
