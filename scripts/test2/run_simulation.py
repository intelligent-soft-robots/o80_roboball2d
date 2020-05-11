import time
import sys
import real_time_tools
import shared_memory
import roboball2d
import roboball2d.factory
import o80
import o80_roboball2d
import world_state_conversions 

# shared memory segment used by o80
segment_id_robot = "sim-robot"
segment_id_ball_gun = "sim-ball-gun"
segment_id_world_state = "sim-world-state"

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
    r2d = roboball2d.factory.get_default(window_size=(400,200))
    
    time_start = time.time()
    previous_ball_gun_id = -1
    previous_action_id = -1
    torques = [0,0,0]
    robot_state = None

    # managing the burst commands sent by
    # the o80 frontend
    burster = o80.Burster(segment_id_robot)

    # converting o80 mirror command into roboball2d robot state
    def _get_robot_state(mirroring_states):
        angles = [mirroring_states.get(index).get_position()
                  for index in range(3)]
        angular_velocities = [mirroring_states.get(index).get_velocity()
                      for index in range(3)]
        robot_state = roboball2d.robot.DefaultRobotState(
            robot_config,
            generalized_coordinates=angles,
            generalized_velocities=angular_velocities)
        return robot_state
    
    running = True
    previous_state = None
    while running:

        # waiting to get order from frontend to perform
        # an iteration
        running = burster.pulse()
        
        # receiving ball-gun shooting command,
        # and shooting if requested
        shooting = ball_gun_backend.pulse()
        if shooting.get(0).get():
            world.reset(None,ball_guns)

        # using o80 mirroring backend to get
        # desired (mirroring) robot states
        # and converting them into roboball2d robot state
        mirroring_states = mirroring_robot_backend.pulse()
        robot_state = _get_robot_state(mirroring_states)

        # running the physics (mirring robot + virtual balls dynamics)
        if previous_state:
            world_state = world.step(None,
                                     mirroring_robot_states=previous_state,
                                     current_time=time.time()-time_start)
        previous_state = robot_state
        

        # using o80 backend to send world state to o80 frontend
        # (world_state needs to be converted first to serializable
        #  o80 world state)
        o80_world_state = o80_roboball2d.OneBallWorldState()
        world_state_conversions.convert(world_state,
                                        o80_world_state)
        world_state_backend.pulse(o80_world_state)        

        # rendering
        if render:
            renderer.render(world_state,[],
                            time_step=1.0/30.0,wait=False)

        
        
if __name__ == '__main__':

    run_simulation()
