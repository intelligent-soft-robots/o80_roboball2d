import roboball2d.physics
import roboball2d.robot
import roboball2d.ball
import roboball2d.ball_gun
import roboball2d.rendering
import o80_roboball2d


class ContextManager:

    def __init__(self,nb_simulated_balls=5,render=True):

        # o80 frontend instances to get states from pseudo real robot,
        # simulation and pseudo real vision
        self._real_robot = o80_roboball2d.RealRobotFrontEnd("real-robot")
        self._simulation = o80_roboball2d.FiveBallsWorldStateFrontEnd("sim-world-state")
        self._vision = o80_roboball2d.BallFrontEnd("vision-ball")

        # robot config 
        self._robot_config = roboball2d.robot.DefaultRobotConfig()
        
        # configs of simulated ball + real ball
        # (first ball will mirror the real ball, and will be green)
        ball_configs = [roboball2d.ball.BallConfig()
                       for _ in range(nb_simulated_balls+1)]
        ball_configs[0].color = [0,1,0]

        # roboball2d simulation
        visible_area_width = 6.0
        self._world = roboball2d.physics.B2World(self._robot_config,
                                                 ball_configs,
                                                 visible_area_width)
        # rendering
        self._render = render
        if(render):
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
                                                                 ball_configs)

        
    def merge(self):

        real_robot_observation = self._real_robot.pulse()
        real_ball_observation = self._vision.pulse()
        sim_world_state_observation = self._simulation.wait_for_next()

        # converting o80 joint states into roboball2d robot state,
        current_robot_joints = real_robot_observation.get_observed_states()
        roboball2d_robot_state = roboball2d.robot.DefaultRobotState(self._robot_config)
        angles = [current_robot_joints.get(dof).get_position()
                  for dof in range(3)]
        velocities = [current_robot_joints.get(dof).get_velocity()
                      for dof in range(3)]
        torques = [current_robot_joints.get(dof).get_torque()
                   for dof in range(3)]
        robot_state = roboball2d.robot.DefaultRobotState(self._robot_config,
                                                         angles,velocities)
        mirroring_robot_states = {0:robot_state}

        # getting real and virtual ball states from o80
        mirroring_ball_states = {}
        real_ball_state = real_ball_observation.get_extended_state()
        mirroring_ball_states[0]= real_ball_state
        sim_world_state = sim_world_state_observation.get_extended_state()
        for index,ball in enumerate(sim_world_state.balls):
            mirroring_ball_states[index+1]=ball
        
        
        # running one state of roboball2d, mirroring real and simulated balls
        context_world_state = self._world.step(torques,
                                               current_time=real_robot_observation.get_time_stamp(),
                                               mirroring_robot_states=mirroring_robot_states,
                                               mirroring_ball_states=mirroring_ball_states)

        # rendering
        if(self._render):
            self._renderer.render(context_world_state,
                                  [],
                                  time_step=1.0/30.0,
                                  wait=False)

        
        # context_world_state encapsulates everything, including contacts
        return angles, velocities, context_world_state
        
