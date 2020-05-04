import time
import sys
import real_time_tools
import shared_memory
import roboball2d.physics
import roboball2d.robot
import roboball2d.ball
import roboball2d.ball_gun
import roboball2d.rendering
import o80_roboball2d
import world_state_conversions 

# configuration
interface_id_robot = "real-robot"
interface_id_ball_gun = "real-ball-gun"
reality_frequency = 2000
class Window:
    def __init__(self):
        self.width = 400
        self.height = 200

# run the pseudo robot (i.e. virtual robot that runs
# iterations at provided frequency, and thus may get unstable)
# and pseudo ball-gun
def run_reality(render=True):

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
        renderer_config.window = Window()
        renderer = roboball2d.rendering.PygletRenderer(renderer_config,
                                                       robot_config,
                                                       ball_config)
        
    # (shared memory) reader and write to communicate
    # with the (c++) driver
    torques_reader = o80_roboball2d.RealRobotReader(interface_id_robot)
    torques_writer = o80_roboball2d.RealRobotWriter(interface_id_robot)
    ball_gun_reader = o80_roboball2d.BallGunReader(interface_id_ball_gun)
    sm_world_state = o80_roboball2d.OneBallWorldState()

    # running the simulation
    time_start = time.time()
    previous_ball_gun_id = -1
    previous_action_id = -1
    torques = [0,0,0]
    running = True
    
    while running :

        try:
        
            # getting from ball gun driver commands for the ball gun
            # to shoot
            ball_gun_action = ball_gun_reader.read_action()
            if all([ball_gun_action.is_valid(),
                    ball_gun_action.should_shoot(),
                    ball_gun_action.id!=previous_ball_gun_id]):
                previous_ball_gun_id=ball_gun_action.id
                # shooting the ball gun in roboball2d sim
                world.reset(None,ball_gun)

            # getting action commands from robot driver
            action = torques_reader.read_action()
            # the action is used if it is valid (i.e. not a dummy initial
            # object) and has torques defined
            should_use_action = previous_action_id!=action.id
            should_use_action = should_use_action and action.is_valid()
            previous_action_id = action.id

            # note : the above implies : for as long there a no new action provided,
            #        the robot keeps applying torques of the last action

            # running one step of the roboball2d sim
            # (setting the current time : having the robot running at 'reality' time
            world_state = world.step(action.get_torques(),
                                     relative_torques=action.are_torques_relative(),
                                     current_time=time.time()-time_start)

            # world.step returns world_state as defined in the roboball2d
            # python package, but roboball2d_interfaces need an instance of
            # o80_roboball2d.roboball2d_interface.WorldState, which is similar but supports
            # serialization, i.e. can be written in shared_memory
            world_state_conversions.convert(world_state,
                                            sm_world_state)

            # sharing the observed world state with rest of the world
            torques_writer.write_world_state(sm_world_state)

            # rendering robot
            if render:
                world_state.ball = None
                renderer.render(world_state,[],time_step=1.0/30.0,wait=False)

            frequency_manager.wait()

        except KeyboardInterrupt:
            running=False
            
        
if __name__ == '__main__':

    run_reality()
