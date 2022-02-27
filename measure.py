import matplotlib.pyplot as plt
import time



class Measure:

    def __init__(self):
        self.times_human = []
        self.times_robot = []
        self.total_times_human = 0
        self.total_times_robot = 0

    def start(self):
        tic = time.perf_counter()
        return tic

    def action_end(self, start_time, agent):
        toc = time.perf_counter()
        ptime = toc - start_time
        if agent == 'human':
            self.total_times_human = start_time + ptime
        else:
            self.total_times_robot = start_time + ptime




