import o80
import numpy as np

path = "/tmp/roboball2d.o80"

attrs = dir(o80.LogAction)
log_actions = [attr for attr in attrs
               if "BACKEND" in attr or "FRONTEND" in attr]

to_str = {int(getattr(o80.LogAction,attr)):attr
          for attr in log_actions}
to_int = {attr:int(getattr(o80.LogAction,attr))
          for attr in log_actions}

class Event:
    def __init__(self,
                 segment_id,
                 log_action,
                 time_stamp=None):
        self.segment_id = segment_id
        self.log_action = log_action
        self.time_stamp = time_stamp
    def __eq__(self,other):
        return all([getattr(self,attr)==getattr(other,attr)
                    for attr in ["segment_id","log_action"]])
    def __str__(self):
        return "\t".join([str(getattr(self,attr))
                          for attr in ["segment_id","log_action","time_stamp"]])

def same_iteration(iter1,iter2):
    if len(iter1)!=len(iter2):
        return False
    for event1,event2 in zip(iter1,iter2):
        if event1!=event2:
            return False
    return True
    
with open(path,"r") as f:
    lines = f.readlines()

events = []
for line in lines:
    segment_id,log_action,time_stamp = line.split("\t")
    log_action = to_str[int(log_action)]
    time_stamp = float(time_stamp)
    events.append(Event(segment_id,
                        log_action,
                        time_stamp))
    
anchor = Event("sim-world-state","FRONTEND_WAIT_END",None)

iterations = []
current_iteration = None

for event in events:
    if event == anchor:
        if current_iteration is not None:
            iterations.append(current_iteration)
        current_iteration = []
    if current_iteration is not None:
        current_iteration.append(event)


        
print("read",len(iterations),"iterations ...")


iteration = iterations[0]
all_the_same = True
for iter in iterations:
    if not same_iteration(iteration,iter):
        all_the_same=False

if all_the_same:
    print("they are all the same !")
else :
    print("WARNING: they are not all the same :(")

print("iteration:")
for event in iteration:
    print("\t",event)

    
class Iteration:
    def __init__(self,events):
        self.duration = abs(events[-1].time_stamp - events[0].time_stamp)
        self.events = events

    def get_event(self,event):
        for event_ in self.events:
            if event_ == event :
                return event_
        return None
    
    @classmethod
    def duration_stats(self,iterations):
        durations = [iter.duration for iter in iterations]
        avg = sum(durations)/float(len(durations))
        std = np.std(durations)
        return avg,std

    @classmethod
    def frequency_stats(self,iterations):
        time_stamps = [iteration.events[0].time_stamp
                       for iteration in iterations]
        periods = []
        for index in range(len(time_stamps)-1):
            periods.append(abs(time_stamps[index]-time_stamps[index+1]))
        frequencies = [1.0/(period/1000.0) for period in periods]
        avg = sum(frequencies)/float(len(frequencies))
        std = np.std(frequencies)
        return avg,std

    @classmethod
    def o80_delays(self,iterations,event_tuple):
        event1 = [iteration.get_event(event_tuple[0])
                  for iteration in iterations]
        event2 = [iteration.get_event(event_tuple[1])
                  for iteration in iterations]
        delays = [abs(e1.time_stamp-e2.time_stamp)
                  for e1,e2 in zip(event1,event2)]
        avg = sum(delays)/float(len(delays))
        std = np.std(delays)
        return avg,std
    
iterations = [Iteration(iteration) for iteration in iterations]

avg,std = Iteration.duration_stats(iterations)
print("iteration average duration:",avg,"| std: ",std)

avg,std = Iteration.frequency_stats(iterations)
print("iteration average frequency:",avg,"| std: ",std)

env_sim_total_o80_delays = (Event("sim-robot","FRONTEND_COMMUNICATE"),
                            Event("sim-robot","BACKEND_WRITE_NEW"))
avg,std = Iteration.o80_delays(iterations,env_sim_total_o80_delays)
print("env/sim o80 total delays (including ball gun delay !):",avg,"| std: ",std)

env_sim_1_o80_delays = (Event("sim-robot","FRONTEND_COMMUNICATE"),
                            Event("sim-robot","BACKEND_READ"))
avg,std = Iteration.o80_delays(iterations,env_sim_1_o80_delays)
print("env/sim o80 delays 1 (including ball gin delay !):",avg,"| std: ",std)

env_sim_bg_o80_delays = (Event("sim-robot","FRONTEND_COMMUNICATE"),
                         Event("sim-ball-gun","BACKEND_READ"))
avg,std = Iteration.o80_delays(iterations,env_sim_bg_o80_delays)
print("env/sim o80 burst delays:",avg,"| std: ",std)

