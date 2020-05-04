import multiprocessing
import time
import sys

import real_time_tools
import shared_memory

import roboball2d

import o80
import roboball2d_interface 

from roboball2d.physics import B2World
from roboball2d.robot import DefaultRobotState

import configuration
import run_support


def run_simulation(config,
                   switch,
                   interface_id_robot,
                   interface_id_ball_gun,
                   render=True):


    nb_balls = config.Balls.nb_balls
    goal = (config.Goal.x_min,config.Goal.x_max,config.Goal.color)

    sim_robot = run_support.Simulation(interface_id_robot,
                                       ball=nb_balls,
                                       robot=True,
                                       ball_gun=True,
                                       render=render,
                                       width=config.Window.width,
                                       height=config.Window.height)

    sim_ball_gun = run_support.Simulation(interface_id_ball_gun,
                                          ball=False,
                                          robot=False,
                                          ball_gun=False,
                                          render=False)

    time_start = time.time()
    previous_ball_gun_id = -1
    previous_action_id = -1
    torques = [0,0,0]
    robot_state = None

    frequency_manager = real_time_tools.FrequencyManager(
        configuration.Interfaces.reality_frequency)

    while run_support.should_run(switch):

        # if ball gun requested to shoot, doing so.
        ball_gun_action = sim_ball_gun.ball_gun_reader.read_action() 
        if all([ball_gun_action.is_valid(),
                ball_gun_action.should_shoot(),
                ball_gun_action.id!=previous_ball_gun_id]):
            previous_ball_gun_id = ball_gun_action.id
            world_state = sim_robot.world.reset(None,sim_robot.ball_gun)
            sim_ball_gun.ball_gun_writer.write_action(
                roboball2d_interface.BallGunAction(False))

        # mirroring the robot based on action provided by the driver
        action = sim_robot.mirror_reader.read_action()
        should_use_action = previous_action_id!=action.id
        should_use_action = should_use_action and action.is_valid()
        
        if should_use_action:

            # mi = [ [angle,angular_velocity], ... ]
            mi = action.get_mirroring_information()
            angles = [m[0] for m in mi]
            angular_velocities = [m[1] for m in mi]
            robot_state = roboball2d.robot.DefaultRobotState(
                sim_robot.robot_config,
                generalized_coordinates=angles,
                generalized_velocities=angular_velocities)
            previous_action_id = action.id
            
            # running the physics (mirring robot + virtual balls dynamics)
            world_state = sim_robot.world.step(None,
                                               mirroring_robot_states=robot_state,
                                               current_time=time.time()-time_start)

            # converting world state to something the driver will
            # be able to use
            sm_world_state = run_support.convert(world_state)

            sim_robot.mirror_writer.write_world_state(sm_world_state)
            
            # rendering
            if sim_robot.renderer:
                sim_robot.renderer.render(world_state,
                                          [goal],
                                          time_step=1.0/30.0,
                                          wait=False)

        frequency_manager.wait()


        
if __name__ == '__main__':

    function = run_simulation
    switch = configuration.Interfaces.switch_simulation
    interface_ids = [configuration.Interfaces.sim_robot,
                     configuration.Interfaces.sim_ball_guns]

    run_support.execute(configuration,
                        function,
                        switch,
                        interface_ids,
                        sys.argv[1:])
