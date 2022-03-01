import matplotlib.pyplot as plt
import time


class Measure:

    def __init__(self):
        self.action_times_human = []
        self.action_times_robot = []
        self.total_times_human = 0
        self.total_times_robot = 0
        self.robot_travel_distance = []
        self.human_travel_distance = []
        self.p_f = []
        self.p_e = []
        self.init_time = None

    def start_time(self):
        tic = time.perf_counter()
        return tic

    def action_end(self, start_time_total, agent, start_time_action=None, idle_time=None, travel_distance=None,
                   action_type=None, action_number=None):
        toc = time.perf_counter()

        if agent == 'human':
            action_time = toc - start_time_total
            self.action_times_human.append(
                (start_time_total - self.init_time, action_time, idle_time, action_type, action_number))

            self.total_times_human += action_time
            self.human_travel_distance.append(travel_distance)
        else:
            planning_time = start_time_action - start_time_total
            action_time = toc - start_time_action
            plan_action_time = toc - start_time_total
            self.total_times_robot += plan_action_time
            self.action_times_robot.append((start_time_total - self.init_time, start_time_action - self.init_time,
                                            planning_time, action_time, plan_action_time, action_type, action_number))
            self.robot_travel_distance.append(travel_distance)

    def human_measures(self, start_time, p_following, p_error):
        self.p_f.append((start_time - self.init_time, p_following))
        self.p_e.append((start_time - self.init_time, p_error))

    def plot_human_measures(self):
        fig, ax = plt.subplots()
        x_val1 = [x[0] for x in self.p_f]
        y_val1 = [x[1] for x in self.p_f]
        x_val2 = [x[0] for x in self.p_e]
        y_val2 = [x[1] for x in self.p_e]
        ax.plot(x_val1, y_val1)
        ax.plot(x_val2, y_val2)
        plt.show()

    def plot_times_actions(self):
        htime_gantt = []
        rtime_gantt = []
        htime_colorface = []
        rtime_colorface = []
        col = {'error1': 'tab:red', 'error2': '#D25E5D', 'allocate': 'tab:blue', 'tray1': '#6BF3FC',
               'normal': 'tab:green', 'tray2': '#27F727', 'idle': '#e7edf3'}
        for ii in self.action_times_human:
            htime_gantt.append((ii[0], ii[1]))
            htime_colorface.append(col[ii[3]])

        for ii in self.action_times_robot:
            rtime_gantt.append((ii[0], ii[4]))
            rtime_colorface.append(col[ii[5]])

        fig, ax = plt.subplots()
        prop = {'edgecolor': 'k'}
        ax.broken_barh(rtime_gantt, (10, 9), facecolors=rtime_colorface, **prop)
        ax.broken_barh(htime_gantt, (20, 9), facecolors=htime_colorface, **prop)
        ax.set_ylim(5, 35)
        ax.set_xlim(0, 200)
        ax.set_xlabel('Time (s)')
        ax.set_yticks([15, 25], labels=['Robot', 'Human'])
        ax.grid(True)
        plt.show()
