import time,math,random,atexit
import real_time_tools
from roboball2d.physics import B2World
from roboball2d.robot.pd_controller import PDController
import o80
import o80_roboball2d


class Orchestrator:

    def __init__(self,reset_real_robot_angles=[0,0,0]):

        self._reset_real_robot_angles = reset_real_robot_angles
        self._reset_controller = PDController()
        
        # related front ends (clients). Used to send commands to the o80 servers
        self._ball_gun = o80_roboball2d.BallGunFrontEnd("real-ball-gun")
        self._real_robot = o80_roboball2d.RealRobotFrontEnd("real-robot")
        self._sim_ball_gun = o80_roboball2d.BallGunFrontEnd("sim-ball-gun")
        self._sim_robot = o80_roboball2d.MirroringFrontEnd("sim-robot")
        self._vision = o80_roboball2d.BallFrontEnd("vision-ball")

        # important: making sure things exit cleanly
        atexit.register(self._clean_exit)
        
        # letting time to start properly
        time.sleep(0.1)


    def apply(self,
              torques = None,
              reset = None,
              shoot = False):

        if reset:
            self._reset_real_robot()

        if shoot:
            self._shoot_real_ball()
            self._shoot_sim_balls()

        if torques is not None:
            self._set_real_torques(torques)
            angles,angular_velocities,_ = self._get_real_robot()
            self._set_mirroring(angles,
                                angular_velocities)
        
    # bringing robot to save place,
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

        # cleaning o80 frontends
        self._sim_robot.final_burst()
        time.sleep(0.1)
        del self._ball_gun
        del self._real_robot
        del self._sim_ball_gun
        del self._sim_robot
        del self._vision


    def _shoot_ball(self,frontend):
        shoot = o80.BoolState(True)
        stop_shoot = o80.BoolState(False)
        frontend.add_command(0,shoot,o80.Mode.OVERWRITE)
        # we shoot only once:
        frontend.add_command(0,stop_shoot,o80.Mode.QUEUE)
        frontend.pulse()
        
    # send a shoot command to the o80 ball gun server
    def _shoot_real_ball(self):
        self._shoot_ball(self._ball_gun)
        
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
        self._sim_robot.burst(1)


        

                
        
