
########################################################
# utilities functions to convert roboball2d.WorldState #
# to o80_roboball2d.WorldState (which is serializable) #
# and vice versa                                       #
########################################################

def item_to_item(item1,item2):
    if item1 is None:
        return
    for attr in ("position","angle",
                 "linear_velocity",
                 "angular_velocity",
                 "torque","desired_torque"):
        v = getattr(item1,attr)
        if v is not None:
            setattr(item2,attr,v)

def _array_to_array(a1,a2):
    list(map(item_to_item,a1,a2))

def _robot_to_robot(r1,r2):
    if r1 is None:
        return
    if r2 is None:
        return
    for attr in ("joints","rods"):
        _array_to_array(getattr(r1,attr),
                        getattr(r2,attr))
    item_to_item(r1.racket,r2.racket)
    
def _convert(w1,w2):
    _robot_to_robot(w1.robot,w2.robot)
    item_to_item(w1.ball,w2.ball)
    if(hasattr(w1,"ball_hits_floor_position")):
        if w1.ball_hits_floor:
            w2.ball_hits_floor = w1.ball_hits_floor_position
        else :
            w2_ball_hits_floor = False
    else :
        if w1.ball_hits_floor:
            w2.ball_hits_floor = True
            w2.ball_hits_floor_position = w1.ball_hits_floor
        else:
            w2.ball_hits_floor = False
    w2.balls_hits_racket = w1.balls_hits_racket
    if w1.t is not None:
        w2.t = w1.t

def convert(world_state_from,
            world_state_to):
    return _convert(world_state_from,
                    world_state_to)

def reverse(sm_world_state,ball_config):
    world_state = WorldState([],[ball_config])
    _convert(sm_world_state,world_state)
    return world_state
