import time,threading
import real_time_tools
from orchestrator import Orchestrator
from context_manager import ContextManager
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

    # uses o80 to send commands to the pseudo real robot / ball gun
    # and the simulated robot / ball gun
    orchestrator = Orchestrator()

    # get synchronized info from pseudo real robot / simulated robot
    # and vision, and runs a simulation step to detect contacts
    context_manager = ContextManager()

    # dummy policy moving robot to random postures
    policy = Policy()

    # init
    orchestrator.apply(torques=[0,0,0])
    
    running = True
    while running:

        # resetting real robot to start position
        # and sending commands to ball guns
        orchestrator.apply(reset=True,shoot=True)

        # resetting context manager: observation
        # during real robot reset can be ignored
        context_manager.reset()
        
        time_start = time.time()

        while time.time()-time_start < 3 :

            try:
                
                # getting robot state from direct observation,
                # and context world state, i.e. world state after
                # an extra simulation step with merged information 
                robot_state,context_world_state = context_manager.merge()

                # getting torques from policy and applying them
                torques = policy.get_torques(robot_state.angles,
                                             robot_state.angular_velocities)
                orchestrator.apply(torques=torques)

                # printing racket contact information
                _print_contacts(context_world_state)
                
                # running at desired frequency
                frequency_manager.wait()
                
            except KeyboardInterrupt:
                running = False


if __name__ == "__main__":

    try :
        run()
    except KeyboardInterrupt:
        pass
