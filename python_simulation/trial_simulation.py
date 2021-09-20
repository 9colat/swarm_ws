import simpy
import random
import statistics

class simulation_environment(object):
    def __init__(self, env, numberof_obstacles):
        self.env = env
        self.obstacle = simpy.Resource(env, numberof_obstacles)
