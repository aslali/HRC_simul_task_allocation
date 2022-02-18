import numpy as np
from scipy.stats import norm
import pulp as plp
from itertools import combinations
import gurobipy
import matplotlib.pyplot as plt
import pickle

import copy


# import PIL as Image


class Planner:
    a = 1

    def __init__(self, init_p_h_alloc, init_p_h_error, init_dt):
        self.p_human_allocation = init_p_h_alloc
        self.p_human_error = init_p_h_error
        self.allocation_time_interval = init_dt
        # self.fig, self.gnt = plt.subplots()
        # self.fig = plt.plot()
        # self.gnt = plt.gca()
        try:
            a_file = open("selection.pkl", "rb")
            output = pickle.load(a_file)
            self.last_selection = output
            # print(output)
        except IOError:
            print("No inital solution: selection")
            self.last_selection = None

        try:
            a_file = open("schedule.pkl", "rb")
            output = pickle.load(a_file)
            self.last_scheduling = output
            # print(output)
        except IOError:
            print("No inital solution: schedule")
            self.last_scheduling = None
        self.previous_htasks = None
        self.previous_rtasks = None

        self.k_hist = 3
        self.alpha_set = [0, 0.2, 0.4, 0.6, 0.8, 1.0]
        self.palpha = []
        self.beta_set = list(np.arange(0, 1.1, 0.1))
        self.pbeta = []
        self.__init_prob()
        self.tm, self.ftm = self.human_error_trans_matrix()

    def __init_prob(self):
        da = [abs(x - self.p_human_allocation) for x in self.alpha_set]
        im = da.index(min(da))
        al = self.alpha_set[im]
        na = len(self.alpha_set)
        q = 5
        for i in self.alpha_set:
            if i == al:
                self.palpha.append(q / (na + q - 1))
            else:
                self.palpha.append(1 / (na + q - 1))

        da = [abs(x - self.p_human_error) for x in self.beta_set]
        im = da.index(min(da))
        al = self.alpha_set[im]
        na = len(self.beta_set)
        q = 5
        for i in self.beta_set:
            if i == al:
                self.pbeta.append(q / (na + q - 1))
            else:
                self.pbeta.append(1 / (na + q - 1))

    def gantt_chart(self, th, tr):
        fig, gnt = plt.subplots()
        gnt.set_ylim(0, 50)
        # Setting X-axis limits
        gnt.set_xlim(0, 300)
        # Setting labels for x-axis and y-axis
        gnt.set_xlabel('seconds since start')
        # Setting ticks on y-axis
        gnt.set_yticks([15, 25, 35])
        # Labelling tickes of y-axis
        gnt.set_yticklabels(['Robot', '', 'Human'])
        # Setting graph attribute
        gnt.grid(True)
        # Declaring a bar in schedule
        gnt.broken_barh(th, (30, 9), facecolors=('olive', 'c'))
        gnt.broken_barh(tr, (10, 9), facecolors=('olive', 'c'))
        plt.show()

    # def gantt_chart(self, th, tr):
    #
    #     # plt.clf()
    #     self.gnt.set_ylim(0, 50)
    #     # Setting X-axis limits
    #     self.gnt.set_xlim(0, 300)
    #     # Setting labels for x-axis and y-axis
    #     self.gnt.set_xlabel('seconds since start')
    #     # Setting ticks on y-axis
    #     self.gnt.set_yticks([15, 25, 35])
    #     # Labelling tickes of y-axis
    #     self.gnt.set_yticklabels(['Robot', '', 'Human'])
    #     # Setting graph attribute
    #     self.gnt.grid(True)
    #     # Declaring a bar in schedule
    #     self.gnt.broken_barh(th, (30, 9), facecolors=('olive', 'c'))
    #     self.gnt.broken_barh(tr, (10, 9), facecolors=('olive', 'c'))
    #     plt.show()
    #     plt.clf()
    #     plt.cla()
    #     plt.close()
    #     self.gnt.clear()

    def task_selection(self, task, hpenalty, rpenalty, error_penalty, prev_sol,
                       save=False):  # todo: add fairness to the cost function


        first_step_available_tasks_tray = []
        for i in task.remained_task_both:
            preced_check = any(j in task.remained_tasks for j in task.task_precedence_dict[i])
            # wrong_act = i in self.human_wrong_actions
            if not preced_check:    # and (i not in task.tasks_allocated_to_human):
                first_step_available_tasks_tray.append(i)

        first_step_available_tasks = list(set(first_step_available_tasks_tray) - set(task.tasks_allocated_to_human))



        nremained = len(task.remained_task_both)
        opt_model = plp.LpProblem(name="MIP_Model")
        x_vars = {i: plp.LpVariable(cat=plp.LpBinary, name="x{0}".format(i)) for i in task.remained_task_both}
        if self.last_selection is not None:
            for i in task.remained_task_both:
                x_vars[i].setInitialValue(self.last_selection["x{0}".format(i)].value())
        z_var = plp.LpVariable('9999', lowBound=0, cat='Continuous')

        alloc_task_human = [0] * task.n_task_total
        for i in task.remained_tasks:
            if i in task.tasks_allocated_to_human:
                alloc_task_human[i] = 1

        alloc_task_robot = [0] * task.n_task_total
        for i in task.remained_tasks:
            if i in task.tasks_allocated_to_robot:
                alloc_task_robot[i] = 1

        constraints = {1: opt_model.addConstraint(
            plp.LpConstraint(
                e=z_var - plp.lpSum(
                    x_vars[i] * (task.t_task_all[i][0] * self.p_human_allocation + hpenalty * (
                            1 - self.p_human_allocation) + hpenalty * alloc_task_robot[i])
                    for i in task.remained_task_both),
                sense=plp.LpConstraintGE, rhs=0, name="constraint_1")),
            2: opt_model.addConstraint(
                plp.LpConstraint(
                    e=z_var - plp.lpSum(
                        (1 - x_vars[i]) * ((task.t_task_all[i][1] * (1 + self.p_human_allocation * alloc_task_human[i])
                                            + self.p_human_error * error_penalty) * 1
                                           + rpenalty * (1 - 1))
                        for i in task.remained_task_both),
                    sense=plp.LpConstraintGE, rhs=0, name="constraint_2"))}
        # mt = max(1, len(first_step_available_tasks) - 1)
        if not task.human_error_tasks_type1:
            if first_step_available_tasks:
                opt_model += (plp.lpSum(x_vars[i] for i in first_step_available_tasks) <= len(first_step_available_tasks) - 1, 'sumb')
            elif first_step_available_tasks_tray:
                opt_model += (plp.lpSum(x_vars[i] for i in first_step_available_tasks_tray) <= len(first_step_available_tasks_tray) - 1, 'sumb')
            else:
                ccccc= 1
        # yprev = [[plp.LpVariable(cat=plp.LpBinary, name='d{sol}_{1}'.format(sol, i)) for i in
        #        task.remained_task_both] for sol in prev_sol.keys()]

        # for sol in prev_sol:
        #     d_vars = {i: plp.LpVariable(cat=plp.LpBinary, name="x{0}".format(i)) for i in task.remained_task_both}

        objective = z_var
        opt_model.sense = plp.LpMinimize
        opt_model.setObjective(objective)
        opt_model.solve(plp.GUROBI_CMD(msg=False, warmStart=True))

        varlist = opt_model.variablesDict()
        self.last_selection = varlist
        if save:
            a_file = open("selection.pkl", "wb")
            pickle.dump(varlist, a_file)
            a_file.close()

        new_task_robot = []
        new_task_human = []
        for i in task.remained_task_both:
            if varlist['x' + str(i)].value() == 0:
                new_task_robot.append(i)
            else:
                new_task_human.append(i)

        for i in task.remained_task_robot_only:
            new_task_robot.append(i)
        for i in task.remained_task_human_only:
            new_task_human.append(i)
        return new_task_human, new_task_robot

    def task_scheduler(self, task_time, human_tasks, robot_tasks, precedence, precedence_type2, remaining_tasks,
                       tasks_human_error, tasks_human_error_type1, tasks_human_error_type2, save=False):
        ntask = len(task_time)
        mlarge = plp.lpSum(task_time) * 100
        nhtask = len(human_tasks)
        nrtask = len(robot_tasks)
        all_task = human_tasks + robot_tasks
        temp_tasks = list(set(all_task) - set(remaining_tasks)) + list(tasks_human_error_type2)
        opt_model = plp.LpProblem(name="sequencingOptim")
        # s_vars = {i: plp.LpVariable(cat=plp.LpInteger, lowBound=0, name="s{0}".format(i)) for i in all_task}
        s_vars = {}
        for i in all_task:
            if i in human_tasks:
                s_vars[i] = plp.LpVariable(cat=plp.LpInteger, lowBound=3, name="s{0}".format(i))
            else:
                s_vars[i] = plp.LpVariable(cat=plp.LpInteger, lowBound=0, name="s{0}".format(i))
        b_vars = {i: plp.LpVariable(cat=plp.LpBinary, name='b{}'.format(i)) for i in robot_tasks}
        yh = [[plp.LpVariable(cat=plp.LpBinary, name='yh{0}_{1}'.format(human_tasks[j], human_tasks[k])) for k in
               range(nhtask)] for j in range(nhtask)]
        # Todo: Check reduction on the number of decision variables
        yr = [[plp.LpVariable(cat=plp.LpBinary, name='yr{0}_{1}'.format(robot_tasks[j], robot_tasks[k])) for k in
               range(nrtask)] for j in range(nrtask)]

        yrh = [[plp.LpVariable(cat=plp.LpBinary, name='yrh{0}_{1}'.format(robot_tasks[j], human_tasks[k])) for k in
                range(nhtask)] for j in range(nrtask)]

        z_var = plp.LpVariable('z', lowBound=0, cat='Integer')

        if self.last_scheduling is not None:

            for i in range(nhtask):
                for j in range(nhtask):
                    pname = 'yh{0}_{1}'.format(human_tasks[i], human_tasks[j])
                    if pname in self.last_scheduling:
                        yh[i][j].setInitialValue(self.last_scheduling[pname].value())

            for i in range(nrtask):
                for j in range(nrtask):
                    pname = 'yr{0}_{1}'.format(robot_tasks[i], robot_tasks[j])
                    if pname in self.last_scheduling:
                        yr[i][j].setInitialValue(self.last_scheduling[pname].value())

        for i in all_task:
            for j in all_task:
                if j != i:
                    if i in precedence:
                        if j in precedence[i]:
                            opt_model += (s_vars[i] - (s_vars[j] + task_time[j]) >= 0, "seq{0}_{1}".format(i, j))
                    elif i in precedence_type2:
                        if j in precedence_type2[i][0]:
                            opt_model += (
                                s_vars[i] - (s_vars[j] + precedence_type2[i][1]) >= 0, "seq{0}_{1}".format(i, j))

        comh = plp.combination(list(range(nhtask)), 2)
        for c1, c2 in comh:
            opt_model += (
                s_vars[human_tasks[c1]] - (s_vars[human_tasks[c2]] + task_time[human_tasks[c2]]) + mlarge * (1 - yh[c1][
                    c2]) >= 0, "B{0}_{1}".format(human_tasks[c1], human_tasks[c2]))

            opt_model += (
                s_vars[human_tasks[c2]] - (s_vars[human_tasks[c1]] + task_time[human_tasks[c1]]) + mlarge * (yh[c1][
                    c2]) >= 0, "Bp{0}_{1}".format(human_tasks[c1], human_tasks[c2]))

        comr = plp.combination(list(range(nrtask)), 2)
        for c1, c2 in comr:
            # if (robot_tasks[c1] in temp_tasks) and (robot_tasks[c2] in temp_tasks):
            #     opt_model += (
            #         s_vars[robot_tasks[c1]] - (s_vars[robot_tasks[c2]] + task_time[
            #             robot_tasks[c2]] + self.allocation_time_interval) + mlarge * (1 - yr[c1][
            #             c2]) >= 0, "B{0}_{1}".format(robot_tasks[c1], robot_tasks[c2]))
            #
            #     opt_model += (
            #         s_vars[robot_tasks[c2]] - (s_vars[robot_tasks[c1]] + task_time[
            #             robot_tasks[c1]] + self.allocation_time_interval) + mlarge * (yr[c1][
            #             c2]) >= 0, "Bp{0}_{1}".format(robot_tasks[c1], robot_tasks[c2]))
            if (robot_tasks[c1] in temp_tasks) and (robot_tasks[c2] not in temp_tasks):
                opt_model += (
                    s_vars[robot_tasks[c2]] - (s_vars[robot_tasks[c1]] + 0) + mlarge * (
                            yr[c1][c2]) >= 0, "B{0}_{1}".format(robot_tasks[c1], robot_tasks[c2]))
                opt_model += (
                    s_vars[robot_tasks[c1]] - (s_vars[robot_tasks[c2]] + task_time[robot_tasks[c2]]) + mlarge * (
                        1 - yr[c1][c2]) >= 0, "Bp{0}_{1}".format(robot_tasks[c1], robot_tasks[c2]))

            elif (robot_tasks[c1] not in temp_tasks) and (robot_tasks[c2] in temp_tasks):
                opt_model += (
                    s_vars[robot_tasks[c1]] - (s_vars[robot_tasks[c2]] + 0) + mlarge * (
                            yr[c1][c2]) >= 0, "B{0}_{1}".format(robot_tasks[c1], robot_tasks[c2]))

                opt_model += (
                    s_vars[robot_tasks[c2]] - (s_vars[robot_tasks[c1]] + task_time[robot_tasks[c1]]) +
                    mlarge * (1-yr[c1][c2]) >= 0, "Bp{0}_{1}".format(robot_tasks[c1], robot_tasks[c2]))
            elif (robot_tasks[c1] not in temp_tasks) and (robot_tasks[c2] not in temp_tasks):
                opt_model += (
                    s_vars[robot_tasks[c1]] - (s_vars[robot_tasks[c2]] + task_time[robot_tasks[c2]]) +
                    mlarge * (1 - yr[c1][c2]) >= 0, "B{0}_{1}".format(robot_tasks[c1], robot_tasks[c2]))

                opt_model += (
                    s_vars[robot_tasks[c2]] - (s_vars[robot_tasks[c1]] + task_time[robot_tasks[c1]]) + mlarge * (yr[c1][
                        c2]) >= 0, "Bp{0}_{1}".format(robot_tasks[c1], robot_tasks[c2]))

        # for i in range(nhtask):
        #     for j in range(nrtask):
        #         jj = robot_tasks[j]
        #         ii = human_tasks[i]
        #         if (jj in precedence_type2) or (jj in tasks_human_error_type2):
        #             opt_model += (s_vars[ii] - (s_vars[jj] + task_time[jj]) + mlarge * yrh[j][i] >= 0,
        #                           "spatial{0}_{1}".format(jj, ii))

        for k in all_task:
            opt_model += (z_var >= s_vars[k] + task_time[k], 'aux{0}'.format(k))

        rt = list(set(robot_tasks) - tasks_human_error)

        rt = list(set(robot_tasks)-set(precedence_type2.keys())-tasks_human_error_type2)
        # if tasks_human_error_type2:
        #     start_zero_alloc = {}
        #     for k in precedence_type2.keys():
        #         start_zero_alloc[k] = plp.LpConstraint(e=s_vars[k] - mlarge * b_vars[k],
        #                                                name='start_zero{}'.format(k), sense=-1,
        #                                                rhs=0)
        #         opt_model.extend(start_zero_alloc[k].makeElasticSubProblem(penalty=100, proportionFreeBound=0.1))
        #         # opt_model += (s_vars[k] <= mlarge * b_vars[k], 'start_zero{}'.format(k))
        #
        #     opt_model += (plp.lpSum(b_vars[i] for i in precedence_type2.keys()) == len(precedence_type2) - 1, 'sumb')
        #
        # else:
        #     for k in rt:
        #         opt_model += (s_vars[k] <= mlarge * b_vars[k], 'start_zero{}'.format(k))
        #     opt_model += (plp.lpSum(b_vars[i] for i in rt) <= len(rt) - 1, 'sumb')

        for k in rt:
            opt_model += (s_vars[k] <= mlarge * b_vars[k], 'start_zero{}'.format(k))
        opt_model += (plp.lpSum(b_vars[i] for i in rt) <= len(rt) - 1, 'sumb')

        objective = z_var
        opt_model.sense = plp.LpMinimize
        opt_model.setObjective(objective)
        # varlist = copy.copy(opt_model.variables())
        self.last_scheduling = opt_model.variablesDict()
        self.previous_htasks = human_tasks
        self.previous_rtasks = robot_tasks
        if save:
            opt_model.solve(plp.GUROBI_CMD(msg=False, warmStart=False))
            a_file = open("schedule.pkl", "wb")
            pickle.dump([self.last_scheduling, human_tasks, robot_tasks], a_file)
            a_file.close()
        else:
            opt_model.solve(plp.GUROBI_CMD(timeLimit=200, msg=False, warmStart=True, gapRel=0.00004))

        is_solution = opt_model.status
        htiming = {}
        rtiming = {}
        if is_solution == 0:
            cccccc = 1
        else:
            self.last_scheduling = opt_model.variablesDict()
            for s in s_vars:
                if s in human_tasks:
                    htiming[s] = s_vars[s].value()
                else:
                    rtiming[s] = s_vars[s].value()

            htiming = {k: v for k, v in sorted(htiming.items(), key=lambda item: item[1])}
            rtiming = {k: v for k, v in sorted(rtiming.items(), key=lambda item: item[1])}
            gnth = [(htiming[x], task_time[x]) for x in htiming]
            gntr = [(rtiming[x], task_time[x]) for x in rtiming]
            gnth.sort(key=lambda x: x[0])
            gntr.sort(key=lambda x: x[0])
            # self.gantt_chart(gnth, gntr)

        return rtiming, htiming, precedence, is_solution

    def adaptability_update(self, human_action, action_history):

        def pyp(a1, a2):
            if a1 == a2:
                return 1.0
            else:
                return 0.0

        def pih(human_action, alpha, action_history, khist):
            if action_history:
                if len(action_history) > khist:
                    histk = action_history[-khist:]
                else:
                    histk = action_history[:]

                nhistk = len(histk)
                n_picked = sum(histk)
                n_not_picked = nhistk - n_picked

                if human_action == 1:
                    p = max(0.001, alpha * (n_picked / nhistk))
                else:
                    p = max(0.001, (1 - alpha) * (n_not_picked / nhistk))
            else:
                p = 1

            return p

        ny = len(self.alpha_set)

        py_temp = self.palpha[:]
        unnorm_p = []
        for j in range(ny):
            p_obs = 1
            pp = 0
            for k in range(ny):
                p1 = pyp(self.alpha_set[j], self.alpha_set[k])
                p2 = pih(human_action, self.alpha_set[k], action_history, self.k_hist)
                pp += p1 * p2 * py_temp[k]
            unnorm_p.append(pp * p_obs)
        self.palpha = [xv / sum(unnorm_p) for xv in unnorm_p]
        # plt.plot(self.alpha_set, self.palpha)
        # plt.show()
        for i in range(ny):
            self.p_human_allocation = sum([a * b for a, b in zip(self.palpha, self.alpha_set)])
        print(self.p_human_allocation)

    def human_error_update(self, human_action, action_history):

        def pyp(y, tm, ftm, y1, y2, ah):
            i1 = y.index(y1)
            i2 = y.index(y2)
            if ah == 1:
                p = ftm[i2, i1]
            else:
                p = tm[i2, i1]

            return p

        def pih(human_action, action_history, khist):
            if action_history:
                if len(action_history) > khist:
                    histk = action_history[-khist:]
                else:
                    histk = action_history[:]

                nhistk = len(histk)
                n_success = sum(histk)
                n_failed = nhistk - n_success
                p_failed = n_failed / nhistk

                if human_action == 1:
                    p = max(0.001, 1 - p_failed)
                else:
                    p = max(0.001, p_failed)
            else:
                p = 1

            return p

        ny = len(self.beta_set)

        py_temp = self.pbeta[:]
        unnorm_p = []
        for j in range(ny):
            p_obs = 1
            pp = 0
            for k in range(ny):
                p1 = pyp(self.beta_set, self.tm, self.ftm, self.beta_set[j], self.beta_set[k], human_action)
                p2 = pih(human_action, action_history, self.k_hist)
                pp += p1 * p2 * py_temp[k]
            unnorm_p.append(pp * p_obs)
        self.pbeta = [xv / sum(unnorm_p) for xv in unnorm_p]
        plt.plot(self.beta_set, self.pbeta)
        plt.show()
        for i in range(ny):
            self.p_human_error = sum([a * b for a, b in zip(self.pbeta, self.beta_set)])

    def human_error_trans_matrix(self):
        gaussian = lambda x, mu, sigma: 1 / (sigma * np.sqrt(2 * np.pi)) * np.exp(-0.5 * ((x - mu) / sigma) ** 2)
        skewedgaussian = lambda x, mu, sigma, alpha: 2 * gaussian(x, mu, sigma) * norm.cdf(alpha * (x - mu) / sigma)
        y = np.linspace(0, 1, 11)
        ny = len(y)
        y = np.append(y, 2)
        tm = np.zeros([ny, ny])
        for j in range(ny):
            for i in range(ny):
                xi = np.arange(y[i], y[i + 1] + 0.001, 0.001)
                yi = skewedgaussian(xi, np.min([1.1, y[j + 1]]), 0.1, 0.1)
                tm[j, i] = np.trapz(yi, xi)
        fig, ax = plt.subplots(1, 2)
        ax[0].imshow(tm, extent=[y[0], y[ny - 1], 0, 1], aspect=1, cmap='gray_r')
        ax[0].set_xticks(y[0:ny])

        ftm = np.flip(tm, 0)
        ftm = np.flip(ftm, 1)
        ax[1].imshow(ftm, extent=[y[0], y[ny - 1], 0, 1], aspect=1, cmap='gray_r')
        plt.show()
        return tm, ftm
