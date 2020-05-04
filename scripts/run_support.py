import multiprocessing
import time
import sys
import shared_memory
import roboball2d_interface
from roboball2d.physics import B2World
from roboball2d.physics import WorldState
from roboball2d.rendering import PygletRenderer
from roboball2d.rendering import RenderingConfig
from roboball2d.robot import DefaultRobotConfig
from roboball2d.robot import DefaultRobotState
from roboball2d.ball import BallConfig
from roboball2d.ball_gun import DefaultBallGun
from roboball2d.robot import DefaultRobotState




##################################
# stopping the running threads   #
# and cleaning the shared memory #
##################################

def should_run(switch):
    value = shared_memory.get_bool("switch",switch)
    return value

def clean_memory(switch, interface_ids):
    shared_memory.clear_shared_memory(switch)
    for interface_id in interface_ids:
        shared_memory.clear_shared_memory(interface_id)

def _stop(switch):
    shared_memory.set_bool("switch",switch,False)
        

###################################
# Read the configuration file     #
# (from same folder this file is) #
###################################
        
def read_config():

    class _Config:
        def __init__(self): pass

    config = _Config()
        
    full_path = os.path.realpath(__file__)
    dir_path = os.path.dirname(full_path)
    config_path = dir_path+os.sep+"configuration.txt"
    if not os.path.isfile(config_path):
        raise Exception("failed to find",config_path)
    with open(config_path,"r") as f:
        for line in f.readline():
            line = line.strip()
            if not line.startswith('#'):
                try:
                    key,value = line.split("\t")
                except :
                    raise Exception("failed to read config line:",line)
                try :
                    value = float(value)
                except :
                    raise Exception("failed to evaluate config line:",line)
                setattr(config,key,value)

    setattr(config,"path",config_path)
            
    return config
    

def get_configuration_attribute(configuration,attr):
    try :
        return getattr(configuration,attr)
    except:
        raise Exception("failed to read "+attr+" from "+
                        configuration.path)
    

##############################################
# Creates all the instances a robot needs    #
# to run and communicate with the c++ driver #
##############################################
    
class Simulation:

    __slots__=[ "robot_config","ball_config",
                "world","renderer","robot_init",
                "ball_gun",
                "torques_reader","torques_writer",
                "mirror_reader","mirror_writer",
                "ball_gun_reader","ball_gun_writer" ]
    
    def __init__(self,interface_id,
                 ball=True,
                 robot=True,
                 ball_gun=False,
                 render=True,
                 render_ball=True,
                 width=None,
                 height=None,
                 location=[0,0]):
        
        for slot in self.__slots__:
            setattr(self,slot,None)

        if robot:
            self.robot_config = DefaultRobotConfig()
            nb_robots=1
        else:
            nb_robots=0

        if ball:
            if ball==True:
                self.ball_config = BallConfig()
                nb_balls=1
            else :
                self.ball_config = [BallConfig() for _
                                   in range(ball)]
                nb_balls=ball
        else:
            nb_balls=1
                
        if ball_gun:
            if ball==True:
                self.ball_gun = DefaultBallGun(self.ball_config)
            else :
                self.ball_gun = [DefaultBallGun(config)
                                for config in self.ball_config]

        visible_area_width = 6.0
        visual_height = 0.05

        if robot or ball:
            self.world = B2World(self.robot_config,
                                 self.ball_config,
                                 visible_area_width)

        # graphics renderer
        if render:
            renderer_config = RenderingConfig(visible_area_width,
                                              visual_height)
            if width is not None:
                renderer_config.window.width=width
            if height is not None:
                renderer_config.window.height=height
            if location is not None:
                renderer_config.window.location = location
            if render_ball:
                self.renderer = PygletRenderer(renderer_config,
                                               self.robot_config,
                                               self.ball_config)
            else :
                self.renderer = PygletRenderer(renderer_config,
                                               self.robot_config,
                                               [])
        if robot:
            self.robot_init = DefaultRobotState(self.robot_config)

        if nb_balls==1 :
            self.torques_reader = roboball2d_interface.TorquesReader(interface_id)
            self.torques_writer = roboball2d_interface.TorquesWriter(interface_id)
            self.mirror_reader = roboball2d_interface.MirrorReader(interface_id)
            self.mirror_writer = roboball2d_interface.MirrorWriter(interface_id)
            self.ball_gun_reader = roboball2d_interface.BallGunReader(interface_id)
            self.ball_gun_writer = roboball2d_interface.BallGunWriter(interface_id)
        elif nb_balls==5:
            self.torques_reader = roboball2d_interface.FiveBallsTorquesReader(interface_id)
            self.torques_writer = roboball2d_interface.FiveBallsTorquesWriter(interface_id)
            self.mirror_reader = roboball2d_interface.FiveBallsMirrorReader(interface_id)
            self.mirror_writer = roboball2d_interface.FiveBallsMirrorWriter(interface_id)
            self.ball_gun_reader = roboball2d_interface.FiveBallsBallGunReader(interface_id)
            self.ball_gun_writer = roboball2d_interface.FiveBallsBallGunWriter(interface_id)
        else :
            raise Exception("roboball2d_interface.run_support: only 1 or 5 balls supported")
        

        
###################################
# starting the simulation process #
###################################
        
def execute(configuration,
            function,
            switch,
            interface_ids,
            args):

    rendering = True
    if "-no-render" in args:
        rendering = False
    stop = False
    if "-stop" in args:
        stop=True

    if stop :
        _stop(switch)
        return
    
    print()
    print("switch:",switch," | interfaces:",*interface_ids)
    print()

    shared_memory.set_bool("switch",switch,True)
    function(configuration,switch,*interface_ids,render=rendering)
    
    clean_memory(switch,interface_ids)
