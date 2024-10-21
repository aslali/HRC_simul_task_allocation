import matplotlib.pyplot as plt
import time
import pickle
import glob
from statistics import mean
import numpy as np
from types import MethodType

from matplotlib.ticker import MaxNLocator

# plt.rcParams['text.usetex'] = True


def load_data(filename):
    with open(filename, 'rb') as f:
        try:
            objdump = pickle.load(f)
            return objdump
        except pickle.UnpicklingError:
            print('Cannot write into object')


def creat_table(casename):
    def get_mean(allvar):
        c = {}
        for i in allvar:
            c[i] = round(mean(allvar[i]), 1)
            sem = round(np.std(allvar[i], ddof=0), 2)
            print('Mean of {0} is {1} and sem is {2}'.format(i, c[i], sem))
        print('{0} & {1} & {2} & {3} & {4} & {5} & {6} & {7} & {8} & {9} & {10}'.format(c['nwrong'], c['twrong'],
                                                                                        c['n_tot_hum_assign'],
                                                                                        c['n_tot_rob_assign'],
                                                                                        c['hum_time'],
                                                                                        c['rob_time'], c['d_h'],
                                                                                        c['d_r'], c['n_h'], c['n_r'],
                                                                                        c['idle_time']))

    allvar = {'twrong': [], 'nwrong': [], 'n_tot_hum_assign': [], 'n_error2': [], 'n_self_hum_assign': [],
              'n_tot_rob_assign': [], 'n_self_rob_assign': [], 'idle_time': [], 'rob_time': [], 'hum_time': [],
              'd_h': [], 'd_r': [], 'n_h': [], 'n_r': []}
    fildir = glob.glob(casename)
    allocate_time = {}
    for i in range(1, 40):
        allocate_time[i] = 0
    maxlen = 0
    for ad in fildir:
        fil = load_data(ad)
        allvar['twrong'].append(fil.twrong)
        allvar['nwrong'].append(fil.nwrong)
        allvar['n_tot_hum_assign'].append(fil.n_tot_hum_assign)
        allvar['n_error2'].append(fil.n_error2)
        allvar['n_self_hum_assign'].append(fil.n_self_hum_assign)
        allvar['n_tot_rob_assign'].append(fil.n_tot_rob_assign)
        allvar['n_self_rob_assign'].append(fil.n_self_rob_assign)
        allvar['idle_time'].append(fil.idle_time)
        allvar['rob_time'].append(fil.rob_time)
        allvar['hum_time'].append(fil.hum_time)
        allvar['d_h'].append(fil.dh)
        # allvar['d_h'].append(sum(fil.human_travel_distance))
        allvar['d_r'].append(fil.dr)
        # allvar['d_r'].append(sum(fil.robot_travel_distance))
        allvar['n_h'].append(len(fil.action_times_human))
        allvar['n_r'].append(len(fil.action_times_robot))

        maxlen = max(len(fil.action_times_robot), maxlen)
        for i in allocate_time:
            if i <= len(fil.action_times_robot):
                if fil.action_times_robot[i - 1][5] == 'allocate':
                    allocate_time[i] += 1

    for i in allocate_time:
        if i > 1:
            allocate_time[i] += allocate_time[i - 1]

    for i in allocate_time:
        allocate_time[i] /= len(fildir)

    get_mean(allvar)
    return allocate_time, maxlen


# measure = load_data('Final_dist/f1e4.pickle')
# measure = load_data('f9_8/f9e8_03.pickle')
# measure.run_all()
# creat_table("fair/*")

# def plot_dists_error(self):
#     nd = len(self.de)
#     nc = 3
#     nr = nd // nc
#     if nd % nc > 0:
#         nr += 1
#     fig, axs = plt.subplots(nr, nc, squeeze=False)
#     # fig.tight_layout()
#
#     i = 0
#     j = 0
#     for p in self.de:
#         axs[i, j].plot(self.de[p]['eset'], self.de[p]['perror'])
#         plt.subplots_adjust(hspace=0.2, wspace=0.2)
#         axs[i, j].set_xlim([0, 1])
#         axs[i, j].set_ylim([0, 1])
#         tit = 't={}'.format(round(p, 2))
#         axs[i, j].set_title(tit, y=1.0, pad=-14, fontsize=13)
#         if j > 0:
#             axs[i, j].set_yticklabels([])
#         elif i == 1:
#             pass
#             # axs[i, j].set_ylabel(r'${P}(p_e)$', fontsize=15)
#
#         if i == nr - 1:
#             if j == 1:
#                 pass
#                 # axs[i, j].set_xlabel(r'$p_e$', fontsize=15)
#
#         else:
#             axs[i, j].set_xticklabels([])
#
#         j += 1
#         if j == nc:
#             j = 0
#             i += 1
#     aa = nc - nd % nc
#     if aa != nc:
#         for ii in range(aa):
#             fig.delaxes(axs[i, j + ii])
#
#     fig.text(0.5, 0.04, r'$p_e$', fontsize=15, ha='center')
#     fig.text(0.04, 0.5, r'$P(p_e)$', fontsize=15, va='center', rotation='vertical')
#     plt.savefig('dist_error.png', format='png', bbox_inches='tight', pad_inches=0)
#     plt.show()
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
#         plt.subplots_adjust(hspace=0.2, wspace=0.2)
#         axs[i, j].set_xlim([0, 1])
#         axs[i, j].set_ylim([0, 1])
#         tit = 't={}'.format(round(p, 2))
#         axs[i, j].set_title(tit, y=1.0, pad=-14, fontsize=13)
#         if j > 0:
#             axs[i, j].set_yticklabels([])
#         elif i == 1:
#             pass
#             # axs[i, j].set_ylabel(r'$P(p_f)$', fontsize=15)
#
#         if i == nr - 1:
#             if j == 1:
#                 pass
#                 # axs[i, j].set_xlabel(r'$p_f$', fontsize=15)
#         else:
#             axs[i, j].set_xticklabels([])
#
#         j += 1
#         if j == nc:
#             j = 0
#             i += 1
#
#     aa = nc - nd % nc
#     if aa != nc:
#         for ii in range(aa):
#             fig.delaxes(axs[i, j + ii])
#
#     fig.text(0.5, 0.04, r'$p_f$', fontsize=15, ha='center')
#     fig.text(0.04, 0.5, r'$P(p_f)$', fontsize=15, va='center', rotation='vertical')
#     plt.savefig('dist_follow.png', format='png', bbox_inches='tight', pad_inches=0)
#     plt.show()
# def plot_human_measures(self):
#     fig, ax = plt.subplots()
#     x_val1 = [x[0] * self.rfast for x in self.p_f]
#     y_val1 = [x[1] for x in self.p_f]
#     x_val2 = [x[0] * self.rfast for x in self.p_e]
#     y_val2 = [x[1] for x in self.p_e]
#     ax.plot(x_val1, y_val1, linewidth=3)
#     ax.plot(x_val2, y_val2, linewidth=3)
#     ax.set_xlabel('time (s)', fontsize=16)
#     ax.set_ylabel(r'$p_e, p_f$')
#     lgd=ax.legend([r'$p_f$', r'$p_e$'], fontsize=15, loc='upper left', frameon=False,
#               bbox_to_anchor=(0.25, -0.1), ncol=2)
#     ax.set_title(r'Expected values of $p_e$ and $p_f$', fontsize=16)
#     ax.set_ylim([0, 1])
#     ax.set_xlim([0, round(x_val1[-1]+10)])
#     ax.set_xticks(range(0, round(x_val1[-1]+10), 20))
#     ax.set_yticks([0.0, 0.2, 0.4, 0.6, 0.8, 1.0])
#     ax.tick_params(axis='x', labelsize=16)
#     ax.tick_params(axis='y', labelsize=16)
#     fig.savefig('samplefigure.png', format='png', bbox_extra_artists=(lgd,), bbox_inches='tight')
#     plt.show()
#
# measure.plot_dists_follow = MethodType(plot_dists_follow, measure)
# measure.plot_dists_error = MethodType(plot_dists_error, measure)
# measure.plot_human_measures = MethodType(plot_human_measures, measure)
# measure.plot_dists_error()
# measure.plot_dists_follow()
# measure.plot_human_measures()

alloc91, l91 = creat_table("f9_1/*")
alloc61, l61 = creat_table("f6_1/*")
alloc31, l31 = creat_table("f3_1/*")

alloc94, l94 = creat_table("f9_4/*")
alloc64, l64 = creat_table("f6_4/*")
alloc34, l34 = creat_table("f3_4/*")

alloc98, l98 = creat_table("f9_8/*")
alloc68, l68 = creat_table("f6_8/*")
alloc38, l38 = creat_table("f3_8/*")

creat_table("fn3_1/*")
creat_table("fn3_4/*")
creat_table("fn3_8/*")

ltx1 = max(l31, l61, l91)
ltx4 = max(l34, l64, l94)
ltx3 = max(l38, l68, l98)
#
x91 = range(1, l91 + 1)
x61 = range(1, l61 + 1)
x31 = range(1, l31 + 1)

x94 = range(1, l94 + 1)
x64 = range(1, l64 + 1)
x34 = range(1, l34 + 1)

x98 = range(1, l98 + 1)  # x98 =
x68 = range(1, 27) #range(1, l68 + 1)
x38 = range(1, 27) # range(1, l38 + 1)
#
y91 = [alloc91[i] for i in x91]
y61 = [alloc61[i] for i in x61]
y31 = [alloc31[i] for i in x31]

xx4 = range(1, ltx4 + 1)
y94 = [alloc94[i] for i in x94]
y64 = [alloc64[i] for i in x64]
y34 = [alloc34[i] for i in x34]

xx8 = range(1, ltx3 + 1)
y98 = [alloc98[i] for i in x98]
y68 = [alloc68[i] for i in x68]
y38 = [alloc38[i] for i in x38]
#
fig, ax = plt.subplots()
ax.plot(x91, y91, 'r', linewidth=2.5)
ax.plot(x61, y61, 'g', linewidth=2.5)
ax.plot(x31, y31, 'b', linewidth=2.5)
ax.set_ylim([0, 12])
ax.set_xlim([0, 27])
ax.xaxis.set_major_locator(MaxNLocator(integer=True))
ax.set_title(r'$P_{error}=0.1$', fontsize=20)
ax.legend([r'$P_{follower}=0.9$', r'$P_{follower}=0.6$'
              , r'$P_{follower}=0.3$'], fontsize=16)
ax.set_xlabel(r'$n$ (Action number)', fontsize=20)
ax.set_ylabel(r'$C_r^h$', fontsize=23)
plt.xticks(np.arange(start=0, stop=28, step=3), fontsize=16)
plt.yticks(fontsize=16)
plt.tight_layout()
fig.savefig('ncml1.eps', format='eps')
plt.show()
# # #
fig, ax = plt.subplots()
ax.plot(x94, y94, 'r', linewidth=2.5)
ax.plot(x64, y64, 'g', linewidth=2.5)
ax.plot(x34, y34, 'b', linewidth=2.5)
ax.set_ylim([0, 12])
ax.set_xlim([0, 27])
ax.set_title(r'$P_{error}=0.4$', fontsize=20)
ax.legend([r'$P_{follower}=0.9$', r'$P_{follower}=0.6$'
              , r'$P_{follower}=0.3$'], fontsize=16)
ax.set_xlabel(r'$n$ (Action number)', fontsize=20)
ax.set_ylabel(r'$C_r^h$', fontsize=23)
plt.xticks(np.arange(start=0, stop=28, step=3), fontsize=16)
plt.yticks(fontsize=16)
plt.tight_layout()
fig.savefig('ncml4.eps', format='eps')
plt.show()
# # #
fig, ax = plt.subplots()
ax.plot(x98, y98, 'r', linewidth=2.5)
ax.plot(x68, y68, 'g', linewidth=2.5)
ax.plot(x38, y38, 'b', linewidth=2.5)
ax.set_ylim([0, 12])
ax.set_xlim([0, 27])
ax.set_title(r'$P_{error}=0.8$', fontsize=20)
ax.legend([r'$P_{follower}=0.9$', r'$P_{follower}=0.6$'
              , r'$P_{follower}=0.3$'], fontsize=16)
ax.set_xlabel(r'$n$ (Action number)', fontsize=20)
ax.set_ylabel(r'$C_r^h$', fontsize=23)
plt.xticks(np.arange(start=0, stop=28, step=3), fontsize=16)
plt.yticks(fontsize=16)
plt.tight_layout()
fig.savefig('ncml8.eps', format='eps')
plt.show()


# #
# lt9x = max(l91, l94, l98)
# lt6x = max(l61, l64, l68)
# lt3x = max(l31, l34, l38)
#
# x9x = range(1, lt9x + 1)
# yy91 = [alloc91[i] for i in x91]
# yy94 = [alloc94[i] for i in x94]
# yy98 = [alloc98[i] for i in x98]
#
# x6x = range(1, lt6x + 1)
# yy61 = [alloc61[i] for i in x61]
# yy64 = [alloc64[i] for i in x64]
# yy68 = [alloc68[i] for i in x68]
#
# x3x = range(1, lt3x + 1)
# yy31 = [alloc31[i] for i in x31]
# yy34 = [alloc34[i] for i in x34]
# yy38 = [alloc38[i] for i in x38]
#
# fig, ax = plt.subplots()
# ax.plot(x91, yy91, 'b')
# ax.plot(x94, yy94, 'g')
# ax.plot(x98, yy98, 'r')
# ax.set_ylim([0, 12])
# ax.set_title(r'$P_{follower}=0.9$', fontsize=15)
# ax.legend([r'$P_{error}=0.1$', r'$P_{error}=0.4$'
#               , r'$P_{error}=0.8$'], fontsize=13)
# ax.set_xlabel(r'$n$ (Action number)', fontsize=15)
# ax.set_ylabel(r'$C_r^h$', fontsize=15)
# fig.savefig('cml9.eps', format='eps', bbox_inches='tight')
# plt.show()
#
# fig, ax = plt.subplots()
# ax.plot(x61, yy61, 'b')
# ax.plot(x64, yy64, 'g')
# ax.plot(x68, yy68, 'r')
# ax.set_ylim([0, 12])
# ax.set_title(r'$P_{follower}=0.6$', fontsize=15)
# ax.legend([r'$P_{error}=0.1$', r'$P_{error}=0.4$'
#               , r'$P_{error}=0.8$'], fontsize=13)
# ax.set_xlabel(r'$n$ (Action number)', fontsize=15)
# ax.set_ylabel(r'$C_r^h$', fontsize=15)
# fig.savefig('cml6.eps', format='eps', bbox_inches='tight')
# plt.show()
#
# fig, ax = plt.subplots()
# ax.plot(x31, yy31, 'b')
# ax.plot(x34, yy34, 'g')
# ax.plot(x38, yy38, 'r')
# ax.set_ylim([0, 12])
# ax.set_title(r'$P_{follower}=0.3$', fontsize=15)
# ax.legend([r'$P_{error}=0.1$', r'$P_{error}=0.4$',
#            r'$P_{error}=0.8$'], fontsize=13)
# ax.set_xlabel(r'$n$ (Action number)', fontsize=15)
# ax.set_ylabel(r'$C_r^h$', fontsize=15)
# fig.savefig('cml3.eps', format='eps', bbox_inches='tight')
# plt.show()
#
#
#
#
# xbar1 = [1, 3, 5]
# xbar2 = [1.65, 3.65, 5.65]
# datanw1 = [1.4, 5, 11.4]
# dataew1 = [1.26, 2.36, 3.2]
# datanw2 = [0.9, 3.1, 8.5]
# dataew2 = [0.88, 1.2, 3.1]
# datana1 = [9.3, 10.4, 11.7]
# dataea1 = [1.25, 1.07, 1.7]
# datana2 = [6.3, 8.5, 10.1]
# dataea2 = [2, 1.18, 1.29]
#
# fig, ax = plt.subplots()
# ba11 = ax.bar(xbar1, datanw1, width=0.5, label='Without adaptation')
# ax.errorbar(xbar1, datanw1, yerr=dataew1, fmt="o", color="k", capsize=7)
# ba12 = ax.bar(xbar2, datanw2, width=0.5, label='With adaptation')
# ax.errorbar(xbar2, datanw2, yerr=dataew2, fmt="o", color="k", capsize=7)
# xlab = [r'$P_{error}=0.1$', r'$P_{error}=0.4$', r'$P_{error}=0.8$']
# ax.set_xticks([1.325, 3.325, 5.325], xlab, fontsize=17)
# ax.set_ylim([0, 15])
# lgd = ax.legend(handles=[ba11, ba12], loc='upper left',
#                 ncol=1, fancybox=False, fontsize=17, frameon=False)
# ax.set_title(r'Human errors for $P_\textit{follow}=0.3$', fontsize=17)
# ax.set_ylabel(r"\# Human errors", fontsize=17)
# fig.savefig('bar1.pdf', bbox_inches='tight')
# plt.show()
#
#
# fig2, ax2 = plt.subplots()
# ba21 = ax2.bar(xbar1, datana1, width=0.5, label='Without adaptation')
# ax2.errorbar(xbar1, datana1, yerr=dataea1, fmt="o", color="k", capsize=7)
# ba22 = ax2.bar(xbar2, datana2, width=0.5, label='With adaptation')
# ax2.errorbar(xbar2, datana2, yerr=dataea2, fmt="o", color="k", capsize=7)
# xlab = [r'$P_{error}=0.1$', r'$P_{error}=0.4$', r'$P_{error}=0.8$']
# ax2.set_xticks([1.325, 3.325, 5.325], xlab, fontsize=17)
# ax2.tick_params(axis='x', labelsize=17)
# ax2.tick_params(axis='y', labelsize=17)
# ax2.set_ylim([0, 15])
# lgd = ax2.legend(handles=[ba21, ba22], loc='upper left',
#                 ncol=1, fancybox=False, fontsize=17, frameon=False)
# ax2.set_title(r'Assigned subtasks by the robot for $P_\textit{follow}=0.3$', fontsize=17)
# ax2.set_ylabel(r"\# Assigned subtasks", fontsize=17)
# plt.show()
# fig2.savefig('bar2.pdf', bbox_inches='tight')



