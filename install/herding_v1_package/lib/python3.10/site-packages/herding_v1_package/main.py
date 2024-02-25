from simulation import Simulation
from results import Results

class Config:
    def __init__(self, dt, tf, k):

        self.dt = dt
        self.time_steps = int(tf / dt)
        self.k = k

if __name__ == '__main__':
    #You probably have to push the time_steps to a node
    #I think dt is a parameter, but ask patrick
    #self.k does not get pushed anywhere
    dt = 0.001
    tf = 5
    k = 1
    config = Config(dt, tf, k)                                                                                        
    simulation = Simulation(config)
    simulation.run()
    Results(simulation)



    #THIS LAUNCHES THE SIM IN PYTHON
    #NOT ROS2
