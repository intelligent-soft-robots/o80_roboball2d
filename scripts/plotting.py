import time
import math
import fyplot
import o80_roboball2d
from functools import partial

def _plot(frontend_robot,frontend_simulation):

    plt = fyplot.Plot("o80_roboball2d",50,(2000,800))
    
    def get_observed_angle(frontend,dof):
        return frontend.read().get_observed_states().get(dof).get_position()

    def get_desired_angle(frontend,dof):
        return frontend.read().get_desired_states().get(dof).get_position()
    
    def get_frequency(frontend):
        return frontend.read().get_frequency()
    
    robot_plots = ( ( partial(get_observed_angle,frontend_robot,0) , (255,0,0) ) ,
                    ( partial(get_observed_angle,frontend_robot,1) , (0,255,0) ) ,
                    ( partial(get_observed_angle,frontend_robot,2) , (0,0,255) ) )

    sim_plots = ( ( partial(get_desired_angle,frontend_simulation,0) , (255,0,0) ) ,
                  ( partial(get_desired_angle,frontend_simulation,1) , (0,255,0) ) ,
                  ( partial(get_desired_angle,frontend_simulation,2) , (0,0,255) ) )

    frequency_plots = ( ( partial(get_frequency,frontend_robot) , (255,0,0) ),
                        ( partial(get_frequency,frontend_simulation) , (0,255,0) ) )

    plt.add_subplot((-1.5,0.2),300,robot_plots)
    plt.add_subplot((-1.5,0.2),300,sim_plots)
    plt.add_subplot((0,2100),300,frequency_plots)
    
    return plt


def run():

    real_robot = o80_roboball2d.RealRobotFrontEnd("real-robot")
    sim_robot = o80_roboball2d.MirroringFrontEnd("sim-robot")

    plot = _plot(real_robot,sim_robot)
    plot.start()

    try :
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass

    plot.stop()
    o80_example.stop_standalone(SEGMENT_ID)

if __name__ == "__main__":
    run()

