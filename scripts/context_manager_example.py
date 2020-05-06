



def run(nb_simulations):

    context_manager = ContextManager(nb_simulations)
    robot_frontend = RealRobotFrontEnd()
    
    while running:

        context = context_manager.wait()

        # computation

        robot_frontend.pulse()


        
        
    
