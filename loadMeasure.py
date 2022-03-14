import matplotlib.pyplot as plt
import time
import pickle
import glob
from statistics import mean

def load_data(filename):
    with open(filename, 'rb') as f:
        try:
            objdump = pickle.load(f)
            return objdump
        except pickle.UnpicklingError:
            print('Cannot write into object')


def creat_table(casename):
    def get_mean(allvar):
        c={}
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
        print(ad, fil.rob_time)
        allvar['hum_time'].append(fil.hum_time)
        allvar['d_h'].append(fil.dh)
        allvar['d_r'].append(fil.dr)
        allvar['n_h'].append(len(fil.action_times_human))
        allvar['n_r'].append(len(fil.action_times_robot))
    print(allvar['n_tot_rob_assign'])
    get_mean(allvar)








# measure = load_data('f9_1/f9e1_1.pickle')
# measure.run_all()
creat_table("f6_8/*")

