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
interface_id_vision = "real-vision"
interface_id_robot = "real-robot"
vision_frequency = 200
class Window:
    def __init__(self):
        self.width = 400
        self.height = 200

# run the pseudo vision (i.e. fixed frequency vision)
# and pseudo ball-gun
def run_vision(render=True):

    # ensures "reality" loops at the desired frequency
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

    # reader to get info from "reality"
    reader = o80_roboball2d.RealRobotReader(interface_id_robot)

    # writer to send info to vision driver
    writer = o80_roboball2d.RealRobotWriter(interface_id_vision)

    running=True
    
    while running:

        try :
            # getting world_state (incl ball info) from "reality"
            # (see run_reality.py in same folder)
            sm_world_state = reader.read_world_state()
            # writting the world_state (incl ball) in the shared memory
            # for the vision driver to read
            writer.write_world_state(sm_world_state)
            # rendering
            world_state = roboball2d.physics.WorldState([],[ball_config]) 
            world_state_conversions.convert(sm_world_state,
                                            world_state)
            renderer.render(world_state,[],time_step=1.0/30.0,wait=False)
            frequency_manager.wait()
        
        except KeyboardInterrupt:
            running=False
            
        
if __name__ == '__main__':

    run_vision()
