import time
import sys

import shared_memory
import real_time_tools

from roboball2d.rendering import PygletRenderer
from roboball2d.rendering import RenderingConfig
from roboball2d.ball import BallConfig

import o80
import roboball2d_interface

import run_support
import configuration

# reads from the shared memory the position of the ball
# as computed by the simulation in run_reality and redirects
# it to the vision driver
def run_vision(configuration,
               switch,
               interface_id_robot,
               interface_id_vision,
               render=True):

    # managing running frequency
    frequency_manager = real_time_tools.FrequencyManager(
        configuration.Interfaces.vision_frequency)
    
    # to simulate vision: we plug directly to the shared memory
    # in which the run_reality process write world_state (originally
    # for the sake of the driver)

    reader = roboball2d_interface.TorquesReader(interface_id_robot)
    writer = roboball2d_interface.TorquesWriter(interface_id_vision)

    ball_config = BallConfig()

    renderer_config = RenderingConfig(configuration.Window.visible_area_width,
                                      configuration.Window.visual_height)
    renderer_config.window.width = configuration.Window.width
    renderer_config.window.height = configuration.Window.height
    
    renderer = PygletRenderer(renderer_config,
                              None,
                              ball_config)

    
    while run_support.should_run(switch):
        
        sm_world_state = reader.read_world_state()
        writer.write_world_state(sm_world_state)
        world_state = run_support.reverse(sm_world_state,ball_config)
        renderer.render(world_state,[],time_step=1.0/30.0,wait=False)
        frequency_manager.wait()
        
        
if __name__ == '__main__':

    function = run_vision
    switch = configuration.Interfaces.switch_vision
    interface_ids = [configuration.Interfaces.real_robot,
                     configuration.Interfaces.vision]

    run_support.execute(configuration,
                        function,
                        switch,
                        interface_ids,
                        sys.argv[1:])
