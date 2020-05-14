import o80
import time,threading
import real_time_tools
from orchestrator import Orchestrator
from policy import Policy


def _print_contacts(context_world_state):
    # context world state is an instance
    # of roboball2d.physics.WorldState
    for index,contact in enumerate(context_world_state.balls_hits_racket):
        if contact:
            if index>0:
                print("virtual ball hit !")
            else:
                print("** real ball hit !")

def run():

    frequency_manager = real_time_tools.FrequencyManager(500)

    # uses o80 to send:
    # torque commands to the pseudo real robot
    # mirroring commands to the simulated robot
    # shoot commands to the simulated ball gun
    orchestrator = Orchestrator()

    # dummy policy moving robot to random postures
    policy = Policy()

    # init
    orchestrator.apply(torques=[0,0,0])

    # logging an iterations
    logger = o80.Logger(5000,"o80logger",False)
    logged = False
    
    running = True
    while running:

        # resetting real robot to start position
        # and sending commands to ball guns
        orchestrator.apply(reset=True,shoot=True)

        # resetting context manager: observation
        # during real robot reset can be ignored
        orchestrator.reset()
        
        time_start = time.time()

        torques = [0.0]*3
        
        while time.time()-time_start < 3 :

            try:

                # sending torques to real robot
                # and mirroring commands to simulation
                orchestrator.apply(torques=torques)
                
                # getting robot state from direct observation,
                # and context world state
                data = orchestrator.observation_manager()
                (ts_robot,robot_state),(ts_context,context_world_state) = data
                
                # getting torques from policy and applying them
                torques = policy.get_torques(robot_state.angles,
                                             robot_state.angular_velocities)


                # printing racket contact information
                _print_contacts(context_world_state)
                
                # running at desired frequency
                frequency_manager.wait()

            except KeyboardInterrupt:
                running = False

        if not logged:
            logger.save("/tmp/roboball2d.o80")
            logged=True
        

if __name__ == "__main__":

    try :
        run()
    except KeyboardInterrupt:
        pass
