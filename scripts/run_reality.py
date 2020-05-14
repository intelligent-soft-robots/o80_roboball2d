import time
import sys
import real_time_tools
import shared_memory
from roboball2d import factory
import o80
import o80_roboball2d
import world_state_conversions 

# o80 shared memory segment id
segment_id_robot = "real-robot"

# pseudo reality will run as fast it can
reality_frequency = 2000

        
def run_reality(render=True):

    # starting a logger
    logger = o80.Logger(5000,"o80logger",True)
    
    # cleanup of previous runs
    o80.clear_shared_memory(segment_id_robot)
    
    # ensures "reality" loops at the desired frequency
    frequency_manager = real_time_tools.FrequencyManager(reality_frequency)

    # creating the roboball2d instances
    r2d = factory.get_default(nb_balls=0,
                              background_color = (1,1,1),
                              ground_color = (1,1,1),
                              window_size=(400,200))
    
    # o80 communication
    # note : robot backend will write new observations only when
    #        a command has been executed
    write_observation_on_new_commands = True
    robot_backend = o80_roboball2d.RealRobotBackEnd(segment_id_robot,
                                                    write_observation_on_new_commands)
    robot_backend.start_logging("o80logger")
    
    # running the simulation
    time_start = time.time()
    torques = [0,0,0]
    running = True

    # initializing roboball2d world_state
    world_state = r2d.world.step([0,0,0],
                                 current_time=time.time()-time_start)

    # function for getting robot joint states (required by o80)
    # from roboball2d.WorldState instance
    def _get_joint_states(world_state):
        robot = world_state.robot
        states = []
        for dof in range(3):
            joint = o80_roboball2d.Joint()
            joint.set_position(robot.joints[dof].angle)
            joint.set_velocity(robot.joints[dof].angular_velocity)
            try:
                joint.set_torque(robot.joints[dof].torque)
            except TypeError: # torque was None
                joint.set_torque(0)
            states.append(joint)
        return states
    
    while running :

        try:

            # o80 pulse : will send current state to
            # frontend, and return desired torques (based
            # on commands sent by the frontend)
            states = _get_joint_states(world_state)
            all_torques = robot_backend.pulse(states)
            torques = [all_torques.get(index).get_torque()
                       for index in range(3)]

            # running one step of roboball2d
            world_state = r2d.world.step(torques,
                                         relative_torques=True,
                                         current_time=time.time()-time_start)

            # rendering robot
            if render:
                world_state.ball = None # robot only, no ball
                world_state.balls = []
                r2d.renderer.render(world_state,[],time_step=1.0/30.0,wait=False)

            # running at desired frequency
            frequency_manager.wait()

        except KeyboardInterrupt:
            running=False
            
        
if __name__ == '__main__':

    run_reality()
