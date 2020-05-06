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

# configuration
segment_id_robot = "sim-robot"
segment_id_ball_gun = "sim-ball-gun"
segment_id_world_state = "sim-world-state"
visible_area_width = 6.0
visual_height = 0.05
n_balls = 5
class Window:
    def __init__(self):
        self.width = 400
        self.height = 200

def run_simulation(render=True):

    # cleanup of previous runs
    o80.clear_shared_memory(segment_id_robot)
    o80.clear_shared_memory(segment_id_ball_gun)
    o80.clear_shared_memory(segment_id_world_state)
    
    # o80 backends
    ball_gun_backend = o80_roboball2d.BallGunBackEnd(segment_id_ball_gun)
    mirroring_robot_backend = o80_roboball2d.MirroringBackEnd(segment_id_robot)
    world_state_backend = o80_roboball2d.FiveBallsWorldStateBackEnd(segment_id_world_state)

    # creating the roboball2d instances
    # for physics and rendering
    ball_configs = [roboball2d.ball.BallConfig() for _ in range(n_balls)]
    robot_config = roboball2d.robot.DefaultRobotConfig()
    ball_guns = [roboball2d.ball_gun.DefaultBallGun(ball_config)
                 for index,ball_config in enumerate(ball_configs)]
    world = roboball2d.physics.B2World(robot_config,
                                       ball_configs,
                                       visible_area_width)
    if render:
        renderer_config = roboball2d.rendering.RenderingConfig(visible_area_width,
                                                               visual_height)
        renderer_config.window = Window()
        renderer = roboball2d.rendering.PygletRenderer(renderer_config,
                                                       robot_config,
                                                       ball_configs)

    
    time_start = time.time()
    previous_ball_gun_id = -1
    previous_action_id = -1
    torques = [0,0,0]
    robot_state = None

    burster = o80.Burster(segment_id_robot)
    running = True
    
    while running:

        # waiting to get order from frontend (env) to perform
        # an iteration
        running = burster.pulse()
        # using ball gun backend to get ball gun
        # shooting command
        shooting = ball_gun_backend.pulse()
        if shooting.get(0).get():
            # shooting balls
            world.reset(None,ball_guns)

        # using mirroring backend to get
        # desired (mirroring) robot states
        mirroring_states = mirroring_robot_backend.pulse()
        angles = [mirroring_states.get(index).get_position()
                  for index in range(3)]
        angular_velocities = [mirroring_states.get(index).get_velocity()
                      for index in range(3)]
        robot_state = roboball2d.robot.DefaultRobotState(
            robot_config,
            generalized_coordinates=angles,
            generalized_velocities=angular_velocities)
            

        # running the physics (mirring robot + virtual balls dynamics)
        world_state = world.step(None,
                                 mirroring_robot_states=robot_state,
                                 current_time=time.time()-time_start)


        # world sharing backend to share current world state
        o80_world_state = o80_roboball2d.FiveBallsWorldState()
        world_state_conversions.convert(world_state,
                                        o80_world_state)
        world_state_backend.pulse(o80_world_state)        

        # rendering
        if render:
            renderer.render(world_state,[],
                            time_step=1.0/30.0,wait=False)

        
        
if __name__ == '__main__':

    run_simulation()
