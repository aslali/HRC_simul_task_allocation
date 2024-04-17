import numpy as np
import matplotlib.pyplot as plt

categories = ['Robot', 'Human']
x = [0.5, 0.9]
values = [19.4, 10]
std_devs = [1.02, 1.1]
bar_width = 0.2
plt.bar(x, values, bar_width, yerr=std_devs, capsize=7, color=['r', 'g'])
plt.ylabel('#Tasks')
plt.xticks(x, categories)
plt.savefig('fairness_tasks_91.eps', format='eps')
plt.show()


values = [18958.4, 19899]  # Mean values
std_devs = [1010.22, 825.12]  # Standard deviations
bar_width = 0.2
plt.bar(x, values, bar_width, yerr=std_devs, capsize=7, color=['r', 'g'])
plt.ylabel('Travel Distance')
plt.xticks(x, categories)
plt.savefig('fairness_dist_91.eps', format='eps')
plt.show()