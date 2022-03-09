import matplotlib.pyplot as plt
import time
import pickle


class Measure:

    def __init__(self, case_name):
        self.action_times_human = []
        self.action_times_robot = []
        self.total_times_human = 0
        self.total_times_robot = 0
        self.robot_travel_distance = []
        self.human_travel_distance = []
        self.p_f = []
        self.p_e = []
        self.de = {}
        self.df = {}
        self.init_time = None
        self.case_name = case_name

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

    def creat_table(self):
        wrong = [x[3] for x in self.action_times_robot if (x[5] == 'error1' or x[5] == 'error2')]
        print('n wrong actions: ', len(wrong))
        print('t wrong actions: ', sum(wrong))

        hassign = [x[3] for x in self.action_times_robot if x[5] == 'tray1']
        print('n assigned by human: ', len(hassign))
        print('t assigned by human: ', sum(hassign))

        rassign = [x[3] for x in self.action_times_robot if x[5] == 'allocate']
        rassign2 = [x[3] for x in self.action_times_robot if x[5] == 'tray2']

        print('n assigned by robot: ', len(rassign), ' -- ', len(rassign2))

        idletime = [x[2] for x in self.action_times_human]
        print('t idle: ', sum(idletime))

        tr = [x[4] for x in self.action_times_robot]
        print('t total robot: ', sum(tr))

        th = [x[1] for x in self.action_times_human]
        print('t total human: ', sum(th))

        print('d total robot: ', sum(self.robot_travel_distance))
        print('d total human: ', sum(self.human_travel_distance))


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

    def human_dist_error(self, start_time, pe, se):
        st = start_time - self.init_time
        self.de[st] = {'perror': pe, 'eset': se}

    def plot_dists_error(self):
        nd = len(self.de)
        nc = 3
        nr = nd // nc
        if nd % nc > 0:
            nr += 1
        fig, axs = plt.subplots(nr, nc, squeeze=False)

        i = 0
        j = 0
        for p in self.de:
            axs[i, j].plot(self.de[p]['eset'], self.de[p]['perror'])
            axs[i, j].set_title(p)
            j += 1
            if j == nc:
                j = 0
                i += 1
        plt.show()

    def human_dist_follow(self, start_time, pf, sf):
        st = start_time - self.init_time
        self.df[st] = {'pfollow': pf, 'fset': sf}

    def plot_dists_follow(self):
        nd = len(self.df)
        nc = 3
        nr = nd // nc
        if nd % nc > 0:
            nr += 1
        fig, axs = plt.subplots(nr, nc, squeeze=False)

        i = 0
        j = 0
        for p in self.df:
            axs[i, j].plot(self.df[p]['fset'], self.df[p]['pfollow'])
            axs[i, j].set_title(p)
            j += 1
            if j == nc:
                j = 0
                i += 1
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

    def run_all(self):
        self.plot_times_actions()
        self.plot_human_measures()
        self.plot_dists_error()
        self.plot_dists_follow()
        self.creat_table()

        filename = self.case_name + ".pickle"
        try:
            with open(filename, "wb") as f:
                pickle.dump(self, f, protocol=pickle.HIGHEST_PROTOCOL)
        except Exception as ex:
            print("Error during pickling object (Possibly unsupported):", ex)
