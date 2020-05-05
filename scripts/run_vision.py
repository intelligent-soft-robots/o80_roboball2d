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
segment_id_real_ball = "real-ball"
segment_id_vision_ball = "vision-ball"
vision_frequency = 100
class Window:
    def __init__(self):
        self.width = 400
        self.height = 200

# run the pseudo vision (i.e. fixed frequency vision)
# and pseudo ball-gun
def run_vision(render=True):

    # cleanup of previous runs
    o80.clear_shared_memory(segment_id_vision_ball)
    
    # ensures loops at the desired frequency
    frequency_manager = real_time_tools.FrequencyManager(vision_frequency)

    # creating the roboball2d instances
    # for rendering
    visible_area_width = 6.0
    visual_height = 0.05
    ball_config = roboball2d.ball.BallConfig()
    if(render):
        renderer_config = roboball2d.rendering.RenderingConfig(visible_area_width,
                                                               visual_height)
        renderer_config.window = Window()
        renderer = roboball2d.rendering.PygletRenderer(renderer_config,
                                                       None,
                                                       ball_config)

    # o80 communication
    real_ball_frontend = o80_roboball2d.BallFrontEnd(segment_id_real_ball)
    vision_ball_backend = o80_roboball2d.BallBackEnd(segment_id_vision_ball)

    # for rendering
    world_state = roboball2d.physics.WorldState([],ball_config)
    
    running=True
    while running:

        try :

            observation = real_ball_frontend.pulse()
            ball = observation.get_extended_state()
            vision_ball_backend.pulse(ball)
            if render:
                world_state_conversions.item_to_item(ball,
                                                     world_state.ball)
                renderer.render(world_state,[],time_step=1.0/30.0,wait=False)
            frequency_manager.wait()
        
        except KeyboardInterrupt:
            running=False
            
        
if __name__ == '__main__':

    run_vision()
