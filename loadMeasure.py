import matplotlib.pyplot as plt
import time
import pickle



# class LoadMeasure:
#
#     # def __init__(self, case_name):
#     #     self.case_name = case_name

def load_data(filename):
    with open(filename, 'rb') as f:
        try:
            objdump = pickle.load(f)
            return objdump
        except pickle.UnpicklingError:
            print('Cannot write into object')


    # def plot_human_measures(self):
    #     fig, ax = plt.subplots()
    #     x_val1 = [x[0] for x in self.p_f]
    #     y_val1 = [x[1] for x in self.p_f]
    #     x_val2 = [x[0] for x in self.p_e]
    #     y_val2 = [x[1] for x in self.p_e]
    #     ax.plot(x_val1, y_val1)
    #     ax.plot(x_val2, y_val2)
    #     plt.show()
    #
    #
    #
    # def plot_dists_error(self):
    #     nd = len(self.de)
    #     nc = 3
    #     nr = nd // nc
    #     if nd % nc > 0:
    #         nr += 1
    #     fig, axs = plt.subplots(nr, nc, squeeze=False)
    #
    #     i = 0
    #     j = 0
    #     for p in self.de:
    #         axs[i, j].plot(self.de[p]['eset'], self.de[p]['perror'])
    #         axs[i, j].set_title(p)
    #         j += 1
    #         if j == nc:
    #             j = 0
    #             i += 1
    #     plt.show()
    #
    #
    #
    # def plot_dists_follow(self):
    #     nd = len(self.df)
    #     nc = 3
    #     nr = nd // nc
    #     if nd % nc > 0:
    #         nr += 1
    #     fig, axs = plt.subplots(nr, nc, squeeze=False)
    #
    #     i = 0
    #     j = 0
    #     for p in self.df:
    #         axs[i, j].plot(self.df[p]['fset'], self.df[p]['pfollow'])
    #         axs[i, j].set_title(p)
    #         j += 1
    #         if j == nc:
    #             j = 0
    #             i += 1
    #     plt.show()
    #
    # def plot_times_actions(self):
    #     htime_gantt = []
    #     rtime_gantt = []
    #     htime_colorface = []
    #     rtime_colorface = []
    #     col = {'error1': 'tab:red', 'error2': '#D25E5D', 'allocate': 'tab:blue', 'tray1': '#6BF3FC',
    #            'normal': 'tab:green', 'tray2': '#27F727', 'idle': '#e7edf3'}
    #     for ii in self.action_times_human:
    #         htime_gantt.append((ii[0], ii[1]))
    #         htime_colorface.append(col[ii[3]])
    #
    #     for ii in self.action_times_robot:
    #         rtime_gantt.append((ii[0], ii[4]))
    #         rtime_colorface.append(col[ii[5]])
    #
    #     fig, ax = plt.subplots()
    #     prop = {'edgecolor': 'k'}
    #     ax.broken_barh(rtime_gantt, (10, 9), facecolors=rtime_colorface, **prop)
    #     ax.broken_barh(htime_gantt, (20, 9), facecolors=htime_colorface, **prop)
    #     ax.set_ylim(5, 35)
    #     ax.set_xlim(0, 200)
    #     ax.set_xlabel('Time (s)')
    #     ax.set_yticks([15, 25], labels=['Robot', 'Human'])
    #     ax.grid(True)
    #     plt.show()
    #
    # def run_all(self):
    #     self.load_data('f4e0.pickle')
    #     self.plot_times_actions()
    #     self.plot_human_measures()
    #     self.plot_dists_error()
    #     self.plot_dists_follow()
measure = load_data('f4e7.pickle')
measure.run_all()
aa = 1