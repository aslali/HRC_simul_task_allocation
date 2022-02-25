# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

import numpy as np
import visualhci
import tasks
import human
import robot

# if __name__ == '__main__':
time_step = 0.064

task_only_human = []
task_only_robot = []
task_both = list(range(20))
# t_only_human = []
# t_only_robot = []
# t_both_human = [10, 10, 10, 10, 10,
#                 20, 20, 20, 20, 20,
#                 20, 20, 20, 20, 20,
#                 10, 10, 10, 10, 10]

# t_both_robot = [2 * x for x in t_both_human]

task_precedence_dict = {0: [], 1: [0], 2: [1], 3: [2], 4: [3],
                        5: [], 6: [5], 7: [6], 8: [7], 9: [8],
                        10: [], 11: [10], 12: [11], 13: [12], 14: [13],
                        15: [], 16: [15], 17: [16], 18: [17], 19: [18]}

task_to_do = {0: (1, 1, 'y'), 1: (1, 2, 'y'), 2: (1, 3, 'y'), 3: (1, 4, 'y'), 4: (1, 5, 'r'),
              5: (2, 1, 'r'), 6: (2, 2, 'r'), 7: (2, 3, 'r'), 8: (2, 4, 'r'), 9: (2, 5, 'g'),
              10: (3, 1, 'g'), 11: (3, 2, 'g'), 12: (3, 3, 'g'), 13: (3, 4, 'g'), 14: (3, 5, 'b'),
              15: (4, 1, 'b'), 16: (4, 2, 'b'), 17: (4, 3, 'b'), 18: (4, 4, 'b'), 19: (4, 5, 'y')}

task = tasks.Task(task_only_human=task_only_human, task_only_robot=task_only_robot, task_both=task_both,
                  task_to_do=task_to_do, task_precedence_dict=task_precedence_dict)
# punish_h = 1.5 * (max(t_both_human) + max(t_both_robot))
# punish_r = 1.5 * (max(t_both_human) + max(t_both_robot))

pattern_col = {}
col1 = ['#00a933', '#ffff00', '#2a6099', '#ff0000']
col2 = ['g', 'y', 'b', 'r']
col = dict(zip(col2, col1))
for i in task_to_do.values():
    pattern_col[(i[0], i[1])] = col[i[2]]

sim_env = visualhci.SHSCPackaging(pattern_col)
human = human.Human(speed=200, task=task, p_conformity=0.7, p_error=0.99, sim_env=sim_env, time_step=time_step)
robot = robot.Robot(speed=150, task=task, human=human, sim_env=sim_env, time_step=time_step)
remained_tasks = task.n_task_total
newAllocation = 1

robot.start()
human.start()


sim_env.root.mainloop()
