#pragma once

#include "o80/pybind11_helper.hpp"
#include "o80_roboball2d/world_state.hpp"

namespace o80_roboball2d {

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

}
