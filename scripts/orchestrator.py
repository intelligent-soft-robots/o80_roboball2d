import time,math,random,atexit
import real_time_tools
import roboball2d
from roboball2d.robot.pd_controller import PDController
import o80
import o80_roboball2d


class Orchestrator:

    def __init__(self,reset_real_robot_angles=[0,0,0]):

        self._reset_real_robot_angles = reset_real_robot_angles
        self._reset_controller = PDController()
        
        # related front ends (clients). Used to send commands to the o80 servers
        self._real_robot = o80_roboball2d.RealRobotFrontEnd("real-robot")
        self._sim_ball_gun = o80_roboball2d.BallGunFrontEnd("sim-ball-gun")
        self._sim_robot = o80_roboball2d.MirroringFrontEnd("sim-robot")
        self._sim_world_state = o80_roboball2d.OneBallWorldStateFrontEnd("sim-world-state")

        # logging all data exchange happening via o80
        #for frontend in (self._real_robot,self._sim_ball_gun,
        #                 self._sim_robot, self._sim_world_state):
        self._real_robot.start_logging("o80logger")
        
        self._robot_config = roboball2d.robot.default_robot_config.DefaultRobotConfig()
        
        # letting time to start properly
        time.sleep(0.1)


    def reset(self):
        # when the pseudo real robot is being reset (between each episode),
        # it generates observations that should be ignored. Solving this here.
        self._real_robot.reset_next_index()
        self._sim_world_state.reset_next_index()


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
        robot_state = roboball2d.robot.DefaultRobotState(self._robot_config,
                                                         angles,velocities)
        time_stamp = real_robot_obs.get_time_stamp()
        return (time_stamp,robot_state)

    
    def _get_context(self):
        # wait for the next observation to be received,
        # and read the state of the simulated balls
        sim_world_state_obs = self._sim_world_state.wait_for_next()
        sim_world_state = sim_world_state_obs.get_extended_state()
        time_stamp = sim_world_state_obs.get_time_stamp()
        return (time_stamp,sim_world_state)

        
    def observation_manager(self):

        real_robot = self._get_robot_state()
        simulation = self._get_context()

        return real_robot,simulation
        
    def apply(self,
              torques = None,
              angles=[0.,0.,0.],
              angular_velocities=[0.,0.,0.],
              reset = None,
              shoot = False):

        # calls a pd controller that has the
        # pseudo real robot go back to its starting
        # position
        if reset:
            self._reset_real_robot()

        # sending shoot commands to the real and
        # simulated ball guns
        if shoot:
            self._shoot_sim_balls()
            self._sim_robot.burst(0)

        # sending torques command to the real robot,
        # and mirroring commands to the simulated robot
        if torques is not None:
            angles,angular_velocities,_ = self._get_real_robot()
            self._set_mirroring(angles,
                                angular_velocities)
            self._sim_robot.burst(0)
            self._set_real_torques(torques)

            
    # bringing robot to safe place,
    # and deleting the o80 frontends (important to be able to restart them)
    def _clean_exit(self):

        print("\n\texiting, please wait ...\n")
        
        # using pd controller to have the robot almost lying
        self._reset_real_robot(500,duration_sec=2.0,refs=[-math.pi/2.5,0,0])

        # removing torques
        joint = o80_roboball2d.Joint()
        joint.set_torque(0)
        for dof in range(3):
            self._real_robot.add_command(dof,
                                         joint,
                                         o80.Mode.OVERWRITE)
        self._real_robot.pulse()
        time.sleep(0.1)

    def _shoot_ball(self,frontend):
        shoot = o80.BoolState(True)
        stop_shoot = o80.BoolState(False)
        frontend.add_command(0,shoot,o80.Mode.OVERWRITE)
        # we shoot only once:
        frontend.add_command(0,stop_shoot,o80.Mode.QUEUE)
        frontend.pulse()
        
    # send a shoot command to the o80 simulated ball guns server
    def _shoot_sim_balls(self):
        self._shoot_ball(self._sim_ball_gun)

    # send torques commands to the o80 real robot
    def _set_real_torques(self,torques):
        for dof,torque in enumerate(torques):
            joint = o80_roboball2d.Joint()
            joint.set_torque(torque)
            self._real_robot.add_command(dof,
                                   joint,
                                   o80.Mode.OVERWRITE)
        self._real_robot.pulse()

    # reading newest state of the robot        
    def _get_real_robot(self):
        joint_states = self._real_robot.pulse().get_observed_states()
        angles = [ joint_states.get(dof).get_position()
                   for dof in range(3) ]
        angular_velocities = [ joint_states.get(dof).get_velocity()
                               for dof in range(3) ]
        torques = [ joint_states.get(dof).get_torque()
                    for dof in range(3) ]
        return angles,angular_velocities,torques

    # runs a pd controller to set the robot back to the
    # initial posture
    def _reset_real_robot(self,frequency=500,duration_sec=2.0,refs=None):
        if refs is None:
            refs = self._reset_real_robot_angles
        time_start = time.time()
        delta_t = 0
        frequency_manager = real_time_tools.FrequencyManager(frequency)
        while delta_t < duration_sec:
            delta_t = time.time()-time_start
            angles,angular_velocities,_ = self._get_real_robot()
            torques = self._reset_controller.get(refs,
                                                 angles,
                                                 angular_velocities)
            for dof,torque in enumerate(torques):
                joint = o80_roboball2d.Joint()
                joint.set_torque(torque)
                self._real_robot.add_command(dof,joint,
                                             o80.Mode.OVERWRITE)
            self._real_robot.pulse()
            frequency_manager.wait()
            
    # sending mirroring commands to simulated robot.
    # and returning simulated world state (i.e. all info
    # managed by the simulation, including simulated balls)
    def _set_mirroring(self,angles,angular_velocities):
        for dof in range(3):
            mirror_joint = o80_roboball2d.MirrorJoint()
            mirror_joint.set(angles[dof],
                             angular_velocities[dof])
            self._sim_robot.add_command(dof,mirror_joint,o80.Mode.OVERWRITE)



        

                
        
