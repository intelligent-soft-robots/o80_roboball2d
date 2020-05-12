import time
import sys
import real_time_tools
import shared_memory
import roboball2d.physics
import roboball2d.robot
import roboball2d.ball
import roboball2d.ball_gun
import roboball2d.rendering
import o80
import o80_roboball2d
import world_state_conversions 

# o80 shared memory segment id
segment_id_robot = "real-robot"
segment_id_ball_gun = "real-ball-gun"
segment_id_real_ball = "real-ball"

# pseudo reality will run as fast it can
reality_frequency = 2000

# roboball2d rendering window size
class Window:
    def __init__(self):
        self.width = 400
        self.height = 200
        
        
def run_reality(render=True):

    # cleanup of previous runs
    o80.clear_shared_memory(segment_id_robot)
    o80.clear_shared_memory(segment_id_ball_gun)
    o80.clear_shared_memory(segment_id_real_ball)
    
    # ensures "reality" loops at the desired frequency
    frequency_manager = real_time_tools.FrequencyManager(reality_frequency)

    # creating the roboball2d instances
    # for physics and rendering
    visible_area_width = 6.0
    visual_height = 0.05
    robot_config = roboball2d.robot.DefaultRobotConfig()
    ball_config = roboball2d.ball.BallConfig()
    ball_gun = roboball2d.ball_gun.DefaultBallGun(ball_config)
    world = roboball2d.physics.B2World(robot_config,
                                       ball_config,
                                       visible_area_width)
    if(render):
        renderer_config = roboball2d.rendering.RenderingConfig(visible_area_width,
                                                               visual_height)
        renderer_config.background_color = (1,1,1,1)
        renderer_config.ground_color = (1,1,1)
        renderer_config.window = Window()
        renderer = roboball2d.rendering.PygletRenderer(renderer_config,
                                                       robot_config,
                                                       ball_config)
        
    # o80 communication
    # note : robot backend will write new observations only when
    #        a command has been executed
    write_observation_on_new_commands = True
    robot_backend = o80_roboball2d.RealRobotBackEnd(segment_id_robot,
                                                    write_observation_on_new_commands)
    ball_gun_backend = o80_roboball2d.BallGunBackEnd(segment_id_ball_gun)
    real_ball_backend = o80_roboball2d.BallBackEnd(segment_id_real_ball)
    
    # running the simulation
    time_start = time.time()
    torques = [0,0,0]
    running = True

    # initializing world_state
    world_state = world.step([0,0,0],
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

            # using ball gun backend to get ball gun
            # shooting command
            shootingStates = ball_gun_backend.pulse()
            if shootingStates.get(0).get():
                # shooting balls
                world.reset(None,ball_gun)

            # o80 pulse : will send current state to
            # frontend, and return desired torques (based
            # on commands sent by the frontend)
            states = _get_joint_states(world_state)
            all_torques = robot_backend.pulse(states)
            torques = [all_torques.get(index).get_torque()
                       for index in range(3)]

            # running one step of roboball2d
            world_state = world.step(torques,
                                     relative_torques=True,
                                     current_time=time.time()-time_start)

            # getting ball info from world state
            # and using the o80 backend to send it to the frontend
            ball = o80_roboball2d.Item()
            world_state_conversions.item_to_item(world_state.ball,ball)
            real_ball_backend.pulse(ball)

            # rendering robot
            if render:
                world_state.ball = None # robot only, no ball
                world_state.balls = []
                renderer.render(world_state,[],time_step=1.0/30.0,wait=False)

            # running at desired frequency
            frequency_manager.wait()

        except KeyboardInterrupt:
            running=False
            
        
if __name__ == '__main__':

    run_reality()
