

class Interfaces:

    # interfaces ids are shared memory keys allowing
    # communication between hardware/simulation and
    # drivers as declared in
    # /include/o80_roboball2d/roboball2d_interface/driver.hpp
    
    real_robot = "real-robot"
    real_ball_gun = "real-ball-gun"
    vision = "vision"
    sim_robot = "sim-robot"
    sim_ball_guns = "sim-ball-guns"

    # switches allows for hardware to receive
    # shut down signals from shared memory
    
    switch_reality = "reality"
    switch_vision = "vision"
    switch_simulation = "simulation"

    # at which frequency the simulation (that pretends
    # to be reality) (attempts to) runs

    reality_frequency = 2000
    
    # at which framerate the vision system will run
    
    vision_frequency = 200
    
    
class Goal:

    # the goal will be drawn on the floor between the two values
    # below. Can also be used for reward computation

    x_min = 2.5
    x_max = 3.5
    color = (0,1,0)
    

class Balls:

    # number of balls the virtual ball gun will shoot
    
    nb_balls = 5

class Window :

    width = 400
    height = 200

    visible_area_width = 6.0
    visual_height = 0.05
