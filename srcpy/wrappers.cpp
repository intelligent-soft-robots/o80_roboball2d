#include "o80/void_state.hpp"
#include "o80_roboball2d/pybind11_helper.hpp"
#include "o80_roboball2d/joint.hpp"
#include "o80_roboball2d/mirror_joint.hpp"
#include "o80_roboball2d/writer.hpp"
#include "o80_roboball2d/reader.hpp"
#include "o80_roboball2d/standalone_torques.hpp"
#include "o80_roboball2d/standalone_ball_gun.hpp"

#define QUEUE 5000

using namespace o80_roboball2d;

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
    
    pybind11::class_<o80_roboball2d::TorquesAction>(m, "TorquesAction")
        .def(pybind11::init<>())
        .def(pybind11::init<double,double,double>())
        .def_readonly("id", &TorquesAction::id)
	.def("is_valid",&TorquesAction::is_valid)
        .def("set_torques", &TorquesAction::set_torques)
	.def("get_torques", &TorquesAction::get_torques)
        .def("are_torques_relative", &TorquesAction::are_torques_relative);

    pybind11::class_<o80_roboball2d::BallGunAction>(m, "BallGunAction")
        .def(pybind11::init<bool>())
        .def_readonly("id", &BallGunAction::id)
	.def("is_valid",&BallGunAction::is_valid)
        .def("set", &BallGunAction::set)
        .def("should_shoot", &BallGunAction::should_shoot);

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
    
    pybind11::class_<o80_roboball2d::Reader<TorquesAction,
					    1,1>>(m,
						  "RealRobotReader")
        .def(pybind11::init<std::string>())
        .def("read_action", &Reader<TorquesAction,1,1>::read_action)
        .def("read_world_state", &Reader<TorquesAction,1,1>::read_world_state);
    
    pybind11::class_<o80_roboball2d::Writer<TorquesAction,
					    1,1>>(m,
						  "RealRobotWriter")
        .def(pybind11::init<std::string>())
        .def("write_world_state", &Writer<TorquesAction,1,1>::write_world_state)
        .def("write_action", &Writer<TorquesAction,1,1>::write_action);

    o80::Pybind11Config config_real_robot;
    config_real_robot.extended_state = false;
    config_real_robot.state = false;
    config_real_robot.states = false;
    config_real_robot.prefix = "RealRobot";
    o80::create_python_bindings<Driver<TorquesAction,1,1>,
                                StandaloneTorques<QUEUE,1,1>,
                                std::string>(m,config_real_robot);

    // for ball-gun control

    o80::Pybind11Config config_ball_gun;
    config_ball_gun.state = false; // added above (BallGunAction)
    config_ball_gun.backend = false; // added below
    config_ball_gun.prefix = "BallGun";
    o80::create_python_bindings<BallGunDriver,
				StandaloneBallGun<QUEUE>,
				std::string>(m,config_ball_gun);

    pybind11::class_<o80_roboball2d::Reader<BallGunAction,1,1>>(
								m, "BallGunReader")
        .def(pybind11::init<std::string>())
        .def("read_action", &Reader<BallGunAction,1,1>::read_action)
        .def("read_world_state", &Reader<BallGunAction,1,1>::read_world_state);
    
    typedef o80::BackEnd<QUEUE,
			 1, // 1 dof
			 o80::BoolState,
			 o80::VoidExtendedState> ball_gun_backend;
    pybind11::class_<ball_gun_backend>(m,"BallGunBackEnd")
	.def(pybind11::init<std::string>())
	.def("pulse",[](ball_gun_backend& bc)
	     {
		 o80::TimePoint time_now = o80::time_now();
		 o80::States<1,o80::BoolState> states;
		 o80::VoidExtendedState void_extended_state;
		 states =
		     bc.pulse(time_now,states,void_extended_state);
		 return states.get(0);
	     });


    // frontend/backend for exchanging mirroring command
    // between python control and simulated robot
    
    o80::Pybind11Config config_mirroring;
    config_mirroring.state = false; // added above (MirrorJoint, done above)
    config_mirroring.backend = false; // added below
    config_mirroring.extended_state = false; // generic VoidExtendedState (unused)
    config_mirroring.prefix = "Mirroring";
    
    o80::create_core_python_bindings<QUEUE,
				     3,
				     MirrorJoint,
				     o80::VoidExtendedState>(m,config_mirroring);
    
    typedef o80::BackEnd<QUEUE,
			 3, // 1 dof
			 MirrorJoint,
			 o80::VoidExtendedState> mirror_backend;
    pybind11::class_<mirror_backend>(m,"MirroringBackEnd")
	.def(pybind11::init<std::string>())
	.def("pulse",[](mirror_backend& bc)
	     {
		 o80::TimePoint time_now = o80::time_now();
		 o80::States<3,MirrorJoint> states;
		 o80::VoidExtendedState void_extended_state;
		 return bc.pulse(time_now,states,void_extended_state);
	     });

    
    // frontend/backend to receive one ball world state

    o80::Pybind11Config config_world_state;
    config_world_state.state = false; // VoidState (unused)
    config_world_state.extended_state = false; // OneBallWorldState (done above)
    config_world_state.prefix = "OneBallWorldState";
    
    o80::create_core_python_bindings<QUEUE,
				     1,
				     o80::VoidState,
				     OneBallWorldState>(m,config_world_state);
    
    // frontend/backend to receive five balls world state

    o80::Pybind11Config config_five_balls_world_state;
    config_five_balls_world_state.state = false; // VoidState (unused)
    config_five_balls_world_state.states = false; // VoidState (unused)
    config_five_balls_world_state.backend = false; // added below
    config_five_balls_world_state.extended_state = false; // FiveBallsWorldsState (done above)
    config_five_balls_world_state.prefix = "FiveBallsWorldState";
    
    o80::create_core_python_bindings<QUEUE,
				     1,
				     o80::VoidState,
				     FiveBallsWorldState>(m,config_five_balls_world_state);
    
    typedef o80::BackEnd<QUEUE,
			 1, // 1 dof
			 o80::VoidState,
			 FiveBallsWorldState> five_balls_backend;
    pybind11::class_<five_balls_backend>(m,"FiveBallsWorldStateBackEnd")
	.def(pybind11::init<std::string>())
	.def("pulse",[](five_balls_backend& bc,FiveBallsWorldState extended_state)
	     {
		 o80::TimePoint time_now = o80::time_now();
		 o80::States<1,o80::VoidState> states;
		 bc.pulse(time_now,states,extended_state);
	     });
    
    
    
    
    
}
