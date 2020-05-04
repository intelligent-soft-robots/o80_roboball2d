cmake_minimum_required(VERSION 2.8.7)

project(o80_roboball2d)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  pybind11_catkin
  robot_interfaces
  shared_memory
  real_time_tools
  o80)

# defined in catkin_package mpi_cmake_modules,
# used to detected for example if using real time computer,
# also host various cmake files
DEFINE_OS()

catkin_package(
 LIBRARIES  o80_roboball2d
 INCLUDE_DIRS include
 CATKIN_DEPENDS shared_memory real_time_tools robot_interfaces o80 )

include_directories(include
  ${catkin_INCLUDE_DIRS} )


################
# Main Library #
################

add_library( o80_roboball2d
  src/torques_action.cpp
  src/ball_gun_action.cpp
  src/mirror_action.cpp
  src/ball_gun_driver.cpp
  src/joint.cpp
  src/mirror_joint)
target_link_libraries(o80_roboball2d ${catkin_LIBRARIES})


#########
# Demos #
#########

add_executable( roboball2d_interface_demo
  demo/roboball2d_interface_demo.cpp )
target_link_libraries(roboball2d_interface_demo
  o80_roboball2d
  ${catkin_LIBRARIES})


###################
# Python bindings #
###################

pybind11_add_module(o80_roboball2d_py
  srcpy/wrappers.cpp)
target_link_libraries(o80_roboball2d_py
  PUBLIC o80_roboball2d ${catkin_LIBRARIES})
set_target_properties(o80_roboball2d_py PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}
  OUTPUT_NAME o80_roboball2d)
install(TARGETS o80_roboball2d_py
  DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})

macro(add_python_module source_file)
  pybind11_add_module(o80_roboball2d_${source_file}
    srcpy/wrappers_${source_file}.cpp)
  target_link_libraries(o80_roboball2d_${source_file}
    PUBLIC o80_roboball2d ${catkin_LIBRARIES})
  set_target_properties(o80_roboball2d_${source_file} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION})
  install(TARGETS o80_roboball2d_${source_file}
    DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})
endmacro(add_python_module source_file)

#add_python_module(torques)
#add_python_module(mirroring)
#add_python_module(ball_gun)
#add_python_module(vision)
#add_python_module(5balls_mirroring)





