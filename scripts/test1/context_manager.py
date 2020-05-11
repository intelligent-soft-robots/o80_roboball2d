import roboball2d.physics
import roboball2d.robot
import roboball2d.ball
import roboball2d.ball_gun
import roboball2d.rendering
import o80_roboball2d


class ContextManager:

    def __init__(self,nb_simulated_balls=5,render=True):
        self._init_o80()
        self._init_roboball2d(nb_simulated_balls)
        self._init_rendering(render)
    
    def _init_o80(self):
        # init the o80 frontend that will be used to receive data
        self._real_robot = o80_roboball2d.RealRobotFrontEnd("real-robot")
        self._simulation = o80_roboball2d.FiveBallsWorldStateFrontEnd("sim-world-state")
        self._vision = o80_roboball2d.BallFrontEnd("vision-ball")

    def _init_roboball2d(self,nb_simulated_balls):
        # init roboball2d world
        self._robot_config = roboball2d.robot.DefaultRobotConfig()
        self._ball_configs = [roboball2d.ball.BallConfig()
                              for _ in range(nb_simulated_balls+1)]
        self._ball_configs[0].color = [0,1,0]
        visible_area_width = 6.0
        self._world = roboball2d.physics.B2World(self._robot_config,
                                                 self._ball_configs,
                                                 visible_area_width)
        
    def _init_rendering(self,render):
        # init roboball2d rendering
        self._render = render
        if(render):
            visible_area_width = 6.0
            visual_height = 0.05
            class Window:
                def __init__(self):
                    self.width = 400
                    self.height = 200
            renderer_config = roboball2d.rendering.RenderingConfig(visible_area_width,
                                                                   visual_height)
            renderer_config.window = Window()
            self._renderer = roboball2d.rendering.PygletRenderer(renderer_config,
                                                                 self._robot_config,
                                                                 self._ball_configs)
            

    def reset(self):
        # when the pseudo real robot is being reset (between each episode),
        # it generates observations that should be ignored. Solving this here.
        self._real_robot.reset_next_index()
        self._simulation.reset_next_index()

    def _get_robot_state(self):
        # wait for the next observation to be received, and read
        # the current robot state from it
        real_robot_obs = self._real_robot.wait_for_next()
        current_robot_joints = real_robot_obs.get_observed_states()
        roboball2d_robot_state = roboball2d.robot.DefaultRobotState(self._robot_config)
        angles = [current_robot_joints.get(dof).get_position()
                  for dof in range(3)]
        velocities = [current_robot_joints.get(dof).get_velocity()
                      for dof in range(3)]
        torques = [current_robot_joints.get(dof).get_torque()
                   for dof in range(3)]
        robot_state = roboball2d.robot.DefaultRobotState(self._robot_config,
                                                         angles,velocities)
        time_stamp = real_robot_obs.get_time_stamp()
        return time_stamp,torques,robot_state

    def _get_ball_state(self):
        # read the latest ball observation (from pseudo vision)
        # and read ball state from it
        # (note: does not "wait" for a new observation, because
        # vision runs at lower frequency and would become a
        # bottleneck)
        real_ball_obs = self._vision.pulse()        
        real_ball_state = real_ball_obs.get_extended_state()
        return real_ball_state

    def _get_sim_balls_state(self):
        # wait for the next observation to be received,
        # and read the state of the simulated balls
        sim_world_state_obs = self._simulation.wait_for_next()
        sim_world_state = sim_world_state_obs.get_extended_state()
        return sim_world_state.balls

    def _rendering(self,world_state):
        # rendering world_state
        if(self._render):
            self._renderer.render(world_state,
                                  [],
                                  time_step=1.0/30.0,
                                  wait=False)

    def merge(self):

        # gets real robot state, real ball state and
        # virtual ball states. Then run a simulation step
        # using these states, and returns the resulting
        # world state
        
        # getting updated real robot state
        time_stamp,torques,real_robot_state = self._get_robot_state()
        mirroring_robot_states = {0:real_robot_state}

        # getting latest real ball state
        real_ball_state = self._get_ball_state()
        mirroring_ball_states = {0:real_ball_state}

        # getting updated virtual balls 
        virtual_balls_state = self._get_sim_balls_state()
        for index,ball in enumerate(virtual_balls_state):
            mirroring_ball_states[index+1]=ball
        
        # running one simulation step 
        context_world_state = self._world.step(torques,
                                               current_time=time_stamp,
                                               mirroring_robot_states=mirroring_robot_states,
                                               mirroring_ball_states=mirroring_ball_states)

        # rendering
        self._rendering(context_world_state)

        # real_robot_state: directly from real robot, with no extra simulation step
        # context_world_state: all information (including contacts) after extra
        #                      simulation step
        return real_robot_state, context_world_state
        
