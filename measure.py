import matplotlib.pyplot as plt
import time


class Measure:

    def __init__(self):
        self.times_human = []
        self.times_robot = []
        self.total_times_human = 0
        self.total_times_robot = 0
        self.robot_travel_distance = []
        self.human_travel_distance = []
        self.p_f = []
        self.p_e = []

    def start_time(self):
        tic = time.perf_counter()
        return tic

    def action_end(self, start_time_total, agent, start_time_action=None, idle_time=None, travel_distance=None,
                   action_type=None, action_number=None):
        toc = time.perf_counter()

        if agent == 'human':
            action_time = toc - start_time_total
            self.times_human.append((start_time_total, start_time_action, action_time, idle_time))
            self.total_times_human += action_time
            self.human_travel_distance.append(travel_distance)
        else:
            planning_time = start_time_action - start_time_total
            action_time = toc - start_time_action
            plan_action_time = toc - start_time_total
            self.total_times_robot += plan_action_time
            self.times_robot.append((start_time_total, start_time_action, planning_time, action_time, plan_action_time))
            self.robot_travel_distance.append(travel_distance)

    def human_measures(self, start_time, p_following, p_error):
        self.p_f.append((start_time, p_following))
        self.p_e.append((start_time, p_error))
