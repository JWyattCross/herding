import numpy as np

class Config:
    def __init__(self, dt, k):
        self.dt = dt
        self.k = k

class Simulation:
    def __init__(self, config):
        self.config = config
        self.agent, self.target = self.initialize()

    def initialize(self):
        target = Target(np.array([0, 0]), self.config)
        agent = Agent(np.array([-10, -10]), self.config, target)
        return agent, target

class Entity:
    def __init__(self, initial_pos, config):
        self.position = np.array([[],[]])#initial_pos #where initial_pos is a np.array([[],[]])
        self.position_previous = self.position #this is to make the tracking error work
        self.dt = config.dt
        self.k = config.k

    def pull(self, ros_pos_node, i): #ros_pos_node needs to be a numpy array shaped like [[x],[y]]
        self.position = ros_pos_node
    
class Agent(Entity):
    def __init__(self, initial_pos, config, target):
        super().__init__(initial_pos, config)
        self.target = target
        self.tracking_error = np.array([[],[]])

    def push(self, i):
        k1 = self.k
        self.tracking_error = self.target.position - self.target.position_previous
        controller = k1*self.tracking_error #Proportional control
        return controller, self.tracking_error #these are both [[],[]] filled with floats

class Target(Entity):
    def push(self, i):
        angular_velocity = 1
        radius = self.position[0,0] #don't know what this does
        omega_t = angular_velocity * self.dt * (i - 1)
        dx = -radius * angular_velocity * np.sin(omega_t)
        dy = radius * angular_velocity * np.cos(omega_t)
        return np.array([dx, dy]) #this value is hollonomic. is converted to lin/ang components in ros node
        #these values are meaningless and need to be scaled/capped to work on real robots