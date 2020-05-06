import time,threading
import real_time_tools
from orchestrator import Orchestrator
from context_manager import ContextManager
from policy import Policy

def run():

    orchestrator = Orchestrator()
    context_manager = ContextManager()
    policy = Policy()

    orchestrator.apply(torques=[0,0,0])
    
    running = True
    while running:

        orchestrator.apply(reset=True,shoot=True)
        time_start = time.time()

        while time.time()-time_start < 3 :

            try:
                angles,angular_velocities,context_world_state = context_manager.merge()
                torques = policy.get_torques(angles,
                                             angular_velocities)
                orchestrator.apply(torques=torques)
            except KeyboardInterrupt:
                running = False
            

if __name__ == "__main__":

    try :
        run()
    except KeyboardInterrupt:
        pass
