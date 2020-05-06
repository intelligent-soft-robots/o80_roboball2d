import time,threading
import real_time_tools
from orchestrator import Orchestrator
from context_manager import ContextManager
from policy import Policy

def run():

    frequency_manager = real_time_tools.FrequencyManager(500)

    orchestrator = Orchestrator()
    context_manager = ContextManager()
    policy = Policy()

    orchestrator.apply(torques=[0,0,0])
    
    running = True
    while running:

        orchestrator.apply(reset=True,shoot=True)
        context_manager.reset()
        time_start = time.time()

        while time.time()-time_start < 3 :

            try:
                robot_state,context_world_state = context_manager.merge()
                torques = policy.get_torques(robot_state.angles,
                                             robot_state.angular_velocities)
                orchestrator.apply(torques=torques)
                frequency_manager.wait()
            except KeyboardInterrupt:
                running = False


if __name__ == "__main__":

    try :
        run()
    except KeyboardInterrupt:
        pass
