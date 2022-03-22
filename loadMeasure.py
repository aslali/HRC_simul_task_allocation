import matplotlib.pyplot as plt
import time
import pickle
import glob
from statistics import mean
from matplotlib.ticker import MaxNLocator

plt.rcParams['text.usetex'] = True


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
            print('Mean of {0} is {1}'.format(i, c[i]))
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
        allvar['d_r'].append(fil.dr)
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

    # get_mean(allvar)
    return allocate_time, maxlen


# measure = load_data('f9_1/f9e1_1.pickle')
# measure.run_all()
alloc91, l91 = creat_table("f9_1/*")
alloc61, l61 = creat_table("f6_1/*")
alloc31, l31 = creat_table("f3_1/*")

alloc94, l94 = creat_table("f9_4/*")
alloc64, l64 = creat_table("f6_4/*")
alloc34, l34 = creat_table("f3_4/*")

alloc98, l98 = creat_table("f9_8/*")
alloc68, l68 = creat_table("f6_8/*")
alloc38, l38 = creat_table("f3_8/*")

ltx1 = max(l31, l61, l91)
ltx4 = max(l34, l64, l94)
ltx3 = max(l38, l68, l98)

x91 = range(1, l91 + 1)
x61 = range(1, l61 + 1)
x31 = range(1, l31 + 1)

x94 = range(1, l94 + 1)
x64 = range(1, l64 + 1)
x34 = range(1, l34 + 1)

x98 = range(1, l98 + 1)  # x98 =
x68 = range(1, 27) #range(1, l68 + 1)
x38 = range(1, 27) # range(1, l38 + 1)

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

fig, ax = plt.subplots()
ax.plot(x91, y91, 'r')
ax.plot(x61, y61, 'g')
ax.plot(x31, y31, 'b')
ax.set_ylim([0, 12])
ax.xaxis.set_major_locator(MaxNLocator(integer=True))
ax.set_title(r'$P_\textit{error}=0.1$', fontsize=15)
ax.legend([r'$P_\textit{follower}=0.9$', r'$P_\textit{follower}=0.6$'
              , r'$P_\textit{follower}=0.3$'], fontsize=13)
ax.set_xlabel(r'$n$ (Action number)', fontsize=15)
ax.set_ylabel(r'$\sum_{i=1}^n{\bigg[a_{r,i} = a^{assign}_{r}\bigg]}$', fontsize=15)
plt.show()

fig, ax = plt.subplots()
ax.plot(x94, y94, 'r')
ax.plot(x64, y64, 'g')
ax.plot(x34, y34, 'b')
ax.set_ylim([0, 12])
ax.set_title(r'$P_\textit{error}=0.4$', fontsize=15)
ax.legend([r'$P_\textit{follower}=0.9$', r'$P_\textit{follower}=0.6$'
              , r'$P_\textit{follower}=0.3$'], fontsize=13)
ax.set_xlabel(r'$n$ (Action number)', fontsize=15)
ax.set_ylabel(r'$\sum_{i=1}^n{\bigg[a_{r,i} = a^{assign}_{r}\bigg]}$', fontsize=15)
plt.show()

fig, ax = plt.subplots()
ax.plot(x98, y98, 'r')
ax.plot(x68, y68, 'g')
ax.plot(x38, y38, 'b')
ax.set_ylim([0, 12])
ax.set_title(r'$P_\textit{error}=0.8$', fontsize=15)
ax.legend([r'$P_\textit{follower}=0.9$', r'$P_\textit{follower}=0.6$'
              , r'$P_\textit{follower}=0.3$'], fontsize=13)
ax.set_xlabel(r'$n$ (Action number)', fontsize=15)
ax.set_ylabel(r'$\sum_{i=1}^n{\bigg[a_{r,i} = a^{assign}_{r}\bigg]}$', fontsize=15)
plt.show()

lt9x = max(l91, l94, l98)
lt6x = max(l61, l64, l68)
lt3x = max(l31, l34, l38)

x9x = range(1, lt9x + 1)
yy91 = [alloc91[i] for i in x91]
yy94 = [alloc94[i] for i in x94]
yy98 = [alloc98[i] for i in x98]

x6x = range(1, lt6x + 1)
yy61 = [alloc61[i] for i in x61]
yy64 = [alloc64[i] for i in x64]
yy68 = [alloc68[i] for i in x68]

x3x = range(1, lt3x + 1)
yy31 = [alloc31[i] for i in x31]
yy34 = [alloc34[i] for i in x34]
yy38 = [alloc38[i] for i in x38]

fig, ax = plt.subplots()
ax.plot(x91, yy91, 'b')
ax.plot(x94, yy94, 'g')
ax.plot(x98, yy98, 'r')
ax.set_ylim([0, 12])
ax.set_title(r'$P_\textit{follower}=0.9$', fontsize=15)
ax.legend([r'$P_\textit{error}=0.1$', r'$P_\textit{error}=0.4$'
              , r'$P_\textit{error}=0.8$'], fontsize=13)
ax.set_xlabel(r'$n$ (Action number)', fontsize=15)
ax.set_ylabel('Cumulative number', fontsize=15)
fig.savefig('cml9.pdf', bbox_inches='tight')
plt.show()
# $\sum_{i=1}^n{\bigg[a_{r,i} = a^{assign}_{r}\bigg]}$

fig, ax = plt.subplots()
ax.plot(x61, yy61, 'b')
ax.plot(x64, yy64, 'g')
ax.plot(x68, yy68, 'r')
ax.set_ylim([0, 12])
ax.set_title(r'$P_\textit{follower}=0.6$', fontsize=15)
ax.legend([r'$P_\textit{error}=0.1$', r'$P_\textit{error}=0.4$'
              , r'$P_\textit{error}=0.8$'], fontsize=13)
ax.set_xlabel(r'$n$ (Action number)', fontsize=15)
ax.set_ylabel('Cumulative number', fontsize=15)
fig.savefig('cml6.pdf', bbox_inches='tight')
plt.show()

fig, ax = plt.subplots()
ax.plot(x31, yy31, 'b')
ax.plot(x34, yy34, 'g')
ax.plot(x38, yy38, 'r')
ax.set_ylim([0, 12])
ax.set_title(r'$P_\textit{follower}=0.3$', fontsize=15)
ax.legend([r'$P_\textit{error}=0.1$', r'$P_\textit{error}=0.4$'
              , r'$P_\textit{error}=0.8$'], fontsize=13)
ax.set_xlabel(r'$n$ (Action number)', fontsize=15)
ax.set_ylabel('Cumulative number', fontsize=15)
fig.savefig('cml3.pdf', bbox_inches='tight')
plt.show()

# fig, ax = plt.subplots()
# xbar1 = [1, 5, 9]
# xbar2 = [2.5, 6.5, 10.5]
# datanw1 = [1.4, 5, 11.4]
# datanw2 = [0.9, 3.1, 8.5]
# datana1 = [9.3, 10.4, 11.7]
# datana2 = [6.3, 8.5, 10.1]
#
# ba11 = ax.bar(xbar2, datanw1, color='darkblue', label=r'$n_r^\textit{assign}$ (Ignoring $p_e$ and $p_f)$', width=1.25)
# ba12 = ax.bar(xbar1, datana1, color='dodgerblue', width=1.25, label=r'$n_h^\textit{wrong}$ (Ignoring $p_e$ and $p_f$)')
# # rects = ax.patches
#
#
# ba21 = ax.bar(xbar2, datanw2, color='darkorange', label=r'$n_r^\textit{assign}$ (Considering $p_e$ and $p_f$)')
# ba22 = ax.bar(xbar1, datana2, color='wheat', label= r'$n_h^\textit{wrong}$ (Considering $p_e$ and $p_f$)')
# xlab = [r'$P_{error}=0.1$', r'$P_{error}=0.4$', r'$P_{error}=0.8$']
# ax.set_xticks([1.75, 5.75, 9.75], xlab)
# ax.set_ylim([0, 13.5])
# lgd = ax.legend(handles=[ba11, ba12, ba21, ba22], loc='lower center',
#           bbox_to_anchor=(0.5, -0.25),
#           ncol=2, fancybox=True
#           )
# ax.set_title(r'$P_\textit{follow}=0.3$')
# labels = [r'$n^{\textit{wrong}}_h$', r'$n_h^\textit{wrong}$',
#           r'$n^{\textit{wrong}}_h$', r'$n_r^\textit{assign}$',
#           r'$n^{\textit{assign}}_r$', r'$n_r^\textit{assign}$']
# # ax.set_ylabel(r'$n_r^\textit{assign}$, $n^{\textit{wrong}}_h$')
# ax.set_ylabel(r"\#")
# # for rect, label in zip(rects, labels):
# #     height = rect.get_height()
# #     ax.text(
# #         rect.get_x() + rect.get_width() / 2, height + 0.35, label, ha="center", va="bottom"
# #     )
# fig.savefig('bar', bbox_extra_artists=([lgd]), bbox_inches='tight')
# plt.show()


xbar1 = [1, 3, 5]
xbar2 = [1.65, 3.65, 5.65]
datanw1 = [1.4, 5, 11.4]
datanw2 = [0.9, 3.1, 8.5]
datana1 = [9.3, 10.4, 11.7]
datana2 = [6.3, 8.5, 10.1]
# color='darkblue'
# color='dodgerblue'
fig, ax = plt.subplots()
ba11 = ax.bar(xbar1, datanw1, width=0.5, label='Without adaptation')
ba12 = ax.bar(xbar2, datanw2, width=0.5, label='With adaptation')
xlab = [r'$P_{error}=0.1$', r'$P_{error}=0.4$', r'$P_{error}=0.8$']
ax.set_xticks([1.325, 3.325, 5.325], xlab, fontsize=15)
ax.set_ylim([0, 13.5])
# bbox_to_anchor=(0.5, -0.25)
lgd = ax.legend(handles=[ba11, ba12], loc='upper left',
                ncol=1, fancybox=True)
ax.set_title(r'Human errors for $P_\textit{follow}=0.3$', fontsize=15)
ax.set_ylabel(r"\#", fontsize=15)
fig.savefig('bar1.pdf', bbox_inches='tight')
plt.show()
# , bbox_extra_artists=([lgd])


fig2, ax2 = plt.subplots()
ba21 = ax2.bar(xbar1, datana1, width=0.5, label='Without adaptation')
ba22 = ax2.bar(xbar2, datana2, width=0.5, label='With adaptation')
xlab = [r'$P_{error}=0.1$', r'$P_{error}=0.4$', r'$P_{error}=0.8$']
ax2.set_xticks([1.325, 3.325, 5.325], xlab, fontsize=17)
ax2.set_yticklabels(fontsize=17)
ax2.set_ylim([0, 13.5])
# bbox_to_anchor=(0.5, -0.25)
lgd = ax2.legend(handles=[ba21, ba22], loc='upper left',
                ncol=1, fancybox=True, fontsize=17)
ax2.set_title(r'Assigned subtasks by the robot for $P_\textit{follow}=0.3$', fontsize=15)
ax2.set_ylabel(r"\# Assigned subtasks", fontsize=17)
plt.show()
fig2.savefig('bar2.pdf', bbox_inches='tight')

# # rects = ax.patches
#
#
# ba21 = ax.bar(xbar2, datanw2, color='darkorange', label=r'$n_r^\textit{assign}$ (Considering $p_e$ and $p_f$)')
# ba22 = ax.bar(xbar1, datana2, color='wheat', label= r'$n_h^\textit{wrong}$ (Considering $p_e$ and $p_f$)')

# # for rect, label in zip(rects, labels):
# #     height = rect.get_height()
# #     ax.text(
# #         rect.get_x() + rect.get_width() / 2, height + 0.35, label, ha="center", va="bottom"
# #     )
# fig.savefig('bar', bbox_extra_artists=([lgd]), bbox_inches='tight')

