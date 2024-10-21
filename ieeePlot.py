import matplotlib.pyplot as plt
import pickle
import glob
import os
from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures
from numpy.polynomial.polynomial import polyval
import numpy as np

from matplotlib.ticker import MaxNLocator

poly = PolynomialFeatures(degree=3, include_bias=False)
poly_reg_model_f = LinearRegression()
poly_reg_model_e = LinearRegression()


def load_data(filename):
    with open(filename, 'rb') as f:
        try:
            objdump = pickle.load(f)
            return objdump
        except pickle.UnpicklingError:
            print('Cannot write into object')

def plot_case_bayes(caseNames):
    fig, ax = plt.subplots()
    data_sets_p_f = []

    common_time = np.linspace(0, 1, 100)
    colors = {
        caseNames[0]: 'blue',
        caseNames[1]: 'orange',
        caseNames[2]: 'green'
    }
    line_styles = {
        caseNames[0]: '-.',
        caseNames[1]: '--',
        caseNames[2]: ':'
    }
    for case in caseNames:
        fildir = glob.glob(case)
        fildir = [f for f in fildir if os.path.isfile(f)]
        data_sets_p_e = []
        for ad in fildir:
            sim_file = load_data(ad)
            x_val1 = [x[0] / sim_file.p_f[-1][0] for x in sim_file.p_f]
            y_val1 = [x[1] for x in sim_file.p_f]
            factors = np.linspace(1, 0.8, len(x_val1))
            # y_val1 = [val * factor for val, factor in zip(y_val1, factors)]
            x_val2 = [x[0] / sim_file.p_e[-1][0] for x in sim_file.p_e]
            y_val2 = [x[1] for x in sim_file.p_e]

            print(ad)
            # print(y_val1[-1])
            print(y_val2[-1])
            print("\n")

            xarr_f = np.array(x_val1)
            yarr_f = np.array(y_val1)

            xarr_e = np.array(x_val2)
            yarr_e = np.array(y_val2)

            xpoly_f = poly.fit_transform(xarr_f.reshape(-1, 1))
            poly_reg_model_f.fit(xpoly_f, yarr_f)
            y_predicted_f = poly_reg_model_f.predict(poly.fit_transform(common_time.reshape(-1, 1)))

            xpoly_e = poly.fit_transform(xarr_e.reshape(-1, 1))
            poly_reg_model_e.fit(xpoly_e, yarr_e)
            y_predicted_e = poly_reg_model_e.predict(poly.fit_transform(common_time.reshape(-1, 1)))

            data_sets_p_f.append(y_predicted_f)
            data_sets_p_e.append(y_predicted_e)

        data_sets_p_e = np.array(data_sets_p_e)
        mean_values_e = np.mean(data_sets_p_e, axis=0)
        std_values_e = np.std(data_sets_p_e, axis=0)
        plt.plot(common_time, mean_values_e, label=case, color=colors[case], linestyle=line_styles[case])
        plt.fill_between(common_time, mean_values_e - std_values_e, mean_values_e + std_values_e,
                         alpha=0.2,
                         color=colors[case],
                         label='±1 Std Dev')

    data_sets_p_f = np.array(data_sets_p_f)
        # Calculate the mean and standard deviation across the datasets
    mean_values_f = np.mean(data_sets_p_f, axis=0)
    std_values_f = np.std(data_sets_p_f, axis=0)
    plt.plot(common_time, mean_values_f, label='pref', color='black')
    plt.fill_between(common_time, mean_values_f - std_values_f, mean_values_f + std_values_f, color='black', alpha=0.2,
                     label='±1 Std Dev')

    ax.set_xlabel('Normalized time', fontsize=20)
    ax.set_ylabel(r'$E(\alpha_f), E(\alpha_e)$', fontsize=20)
    # lgd=ax.legend([r'$\alpha_f$', r'$\alpha_e$'], fontsize=15, loc='upper left', frameon=False,
    # #           bbox_to_anchor=(0.25, -0.1), ncol=2)
    handles, labels = plt.gca().get_legend_handles_labels()
    handles = [handle for handle, label in zip(handles, labels) if label in caseNames+['pref']]
    labels = [label for label in labels if label in caseNames+['pref']]
    handles = [handles[3]] + handles[0:3]
    labels = [r'$\alpha_{f}$ ($P_{follow}=0.9$)', r'$\alpha_{e}$ ($P_{error}=0.1$)',
              r'$\alpha_{e}$ ($P_{error}=0.4$)', r'$\alpha_{e}$ ($P_{error}=0.8$)']
    # plt.legend()

    lgd = ax.legend(handles, labels, fontsize=15, loc='upper left', frameon=False,
                    ncol=2)
    bbox_to_anchor=(0.25, 1)
    ax.set_title(r'Expected $\alpha_f$ and $\alpha_e$ over time ($P_{follow}=0.3$)', fontsize=19)
    ax.set_ylim([0, 1.3])
    ax.set_xlim([0, 1])
    # ax.set_xticks(range(0, round(x_val1[-1]+10), 20))
    ax.set_yticks([0.0, 0.2, 0.4, 0.6, 0.8, 1.0])
    ax.tick_params(axis='x', labelsize=16)
    ax.tick_params(axis='y', labelsize=16)
    plt.tight_layout()
    fig.savefig('pfollow3_simul.pdf', format='pdf', )
    plt.show()


plot_case_bayes(['f3_1/*', 'f3_4/*', 'f3_8/*'])
# plot_case_bayes(['f6_1/*', 'f6_4/*', 'f6_8/*'])
# plot_case_bayes(['f9_1/*', 'f9_4/*', 'f9_8/*'])