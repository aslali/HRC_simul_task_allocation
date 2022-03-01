import math
import numpy as np
import time
import threading
import planner
import copy


class Robot(threading.Thread):
    speed = 1.0

    def __init__(self, task, speed, human, sim_env, time_step, measure):
        threading.Thread.__init__(self)

        self.p_human_allocation = 0.8
        self.p_human_error = 0.1
        self.allocation_time_interval = 0
        self.planner = planner.Planner(self.p_human_allocation, self.p_human_error, self.allocation_time_interval)
        self.time_step = time_step
        self.task = task
        self.human = human
        self.sim_env = sim_env
        self.rpoints = sim_env.rpoints
        self.hpoints = sim_env.hpoints
        self.speed = speed
        self.rob_slopdist = {}
        self.hum_slopdist = {}
        self.all_allocated_tasks = []
        self.cur_allocated_tasks = []
        self.interaction_history = []
        self.pre_human_tasks_done = []
        self.pre_human_wrong_actions = []
        self.human_accuracy_history = []
        self.slop_distance()
        self.tasks_required_time()
        self.save_init_sol = False
        self.safe_dist_hr = 180
        self.measure = measure


        # print(self.rob_slopdist)

    def slop_distance(self):
        cases = ['W1', 'W2', 'W3', 'W4']
        for i in cases:
            s = np.array(self.rpoints['T'])
            g = np.array(self.rpoints[i])
            m = (s[1] - g[1]) / (s[0] - g[0])
            d2 = (s[0] - g[0]) ** 2 + (s[1] - g[1]) ** 2
            d = np.sqrt(d2)
            name = 'T' + i
            self.rob_slopdist[name] = (d, m)
        for i in cases:
            s = np.array(self.hpoints['T'])
            g = np.array(self.hpoints[i])
            m = (s[1] - g[1]) / (s[0] - g[0])
            d2 = (s[0] - g[0]) ** 2 + (s[1] - g[1]) ** 2
            d = np.sqrt(d2)
            name = 'T' + i
            self.hum_slopdist[name] = (d, m)

    def tasks_required_time(self):
        for i in range(5):
            self.task.t_task_all[i] = (self.hum_slopdist['TW1'][0] * 2 / self.human.speed,
                                       self.rob_slopdist['TW1'][0] * 2 / self.speed + 2)
        for i in range(5, 10):
            self.task.t_task_all[i] = (self.hum_slopdist['TW2'][0] * 2 / self.human.speed,
                                       self.rob_slopdist['TW2'][0] * 2 / self.speed + 2)
        for i in range(10, 15):
            self.task.t_task_all[i] = (self.hum_slopdist['TW3'][0] * 2 / self.human.speed,
                                       self.rob_slopdist['TW3'][0] * 2 / self.speed + 2)
        for i in range(15, 20):
            self.task.t_task_all[i] = (self.hum_slopdist['TW4'][0] * 2 / self.human.speed,
                                       self.rob_slopdist['TW4'][0] * 2 / self.speed + 2)

    def action_selection(self):
        pass

    # def robot_move(self, start, goal):
    #     s = np.array(self.rpoints[start])
    #     g = np.array(self.rpoints[goal])
    #     name = start + goal if start + goal in self.rob_slopdist else goal + start
    #     t = round(self.rob_slopdist[name][0] / self.speed)
    #     # print(t)
    #     x = np.linspace(s[0], g[0], round(1 / self.time_step) * t)
    #     y = self.rob_slopdist[name][1] * (x - s[0]) + s[1]
    #     xcur = s[0]
    #     count = 1
    #     if not (self.sim_env.human_pos[0] < self.sim_env.table_h + 400) or not (xcur < self.sim_env.table_h + 400) \
    #             or not (xcur > self.sim_env.human_pos[0]):
    #
    #         while abs(xcur - g[0]) > 0.0:
    #             xcur = x[count]
    #             ycur = y[count]
    #             self.sim_env.move_human_robot([xcur, ycur], 'robot')
    #             time.sleep(self.time_step)
    #             count += 1

    def robot_move_apf(self, start, goal):
        travel_dist = 0
        s = np.array(self.rpoints[start])
        g = np.array(self.rpoints[goal])
        name = start + goal if start + goal in self.rob_slopdist else goal + start
        t = round(self.rob_slopdist[name][0] / self.speed)
        # x = np.linspace(s[0], g[0], round(1 / self.time_step) * t)
        # y = self.rob_slopdist[name][1] * (x - s[0]) + s[1]
        xcur = s[0]
        ycur = s[1]
        hum_pre_pos_x = None
        hum_pre_pos_y = None
        rd = 50
        if not (self.sim_env.human_pos[0] < self.sim_env.table_h + 400) or not (
                xcur < self.sim_env.table_h + 400) \
                or not (xcur > self.sim_env.human_pos[0]):

            while abs(xcur - g[0]) > 10:
                drh2 = (xcur - self.sim_env.human_pos[0]) ** 2 + (ycur - self.sim_env.human_pos[1]) ** 2
                drh = math.sqrt(drh2) - 50

                alpha = math.atan2(g[1] - ycur, g[0] - xcur)
                xatt = math.cos(alpha)
                yatt = math.sin(alpha)
                if drh < self.safe_dist_hr:

                    c1 = (1 / drh - 1 / self.safe_dist_hr) / drh ** 3
                    c2 = 2 * 10 ** 4
                    dxrep = (xcur - self.sim_env.human_pos[0])
                    dyrep = (ycur - self.sim_env.human_pos[1])
                    beta = math.atan2(dyrep, dxrep)
                    xrep = c1 * c2 * math.cos(beta)
                    yrep = c1 * c2 * math.sin(beta)

                    if hum_pre_pos_x is not None:
                        dummy_obs_ang = math.atan2(self.sim_env.human_pos[1] - hum_pre_pos_y,
                                                   self.sim_env.human_pos[0] - hum_pre_pos_x)
                        dummy_x = self.sim_env.human_pos[0] + (25 + rd) * math.cos(dummy_obs_ang)
                        dummy_y = self.sim_env.human_pos[1] + (25 + rd) * math.sin(dummy_obs_ang)
                        # self.sim_env.canvas.create_oval(dummy_x, dummy_y, dummy_x + 2 * rd, dummy_y + 2 * rd)
                        drd2 = (dummy_x - xcur) ** 2 + (dummy_y - ycur) ** 2
                        drd = math.sqrt(drd2) - rd - 25
                        cd1 = (1 / drd - 1 / self.safe_dist_hr) / drd ** 3
                        cd2 = 5 * 10 ** 7
                        ddxrep = (xcur - dummy_x)
                        ddyrep = (ycur - dummy_y)
                        dbeta = math.atan2(ddyrep, ddxrep)
                        xrepd = cd1 * cd2 * math.cos(dbeta)
                        yrepd = cd1 * cd2 * math.sin(dbeta)
                        xrep += xrepd
                        yrep += yrepd

                else:
                    xrep = 0
                    yrep = 0

                speedx = np.sign(xatt + xrep) * min(250, abs(xatt + xrep) * self.speed)
                speedy = np.sign(yatt + yrep) * min(250, abs(yatt + yrep) * self.speed)
                dx = speedx * self.time_step
                dy = speedy * self.time_step
                xcur += dx
                ycur += dy
                travel_dist += math.sqrt(dx ** 2 + dy ** 2)
                hum_pre_pos_x = self.sim_env.human_pos[0]
                hum_pre_pos_y = self.sim_env.human_pos[1]
                self.sim_env.move_human_robot([xcur, ycur], 'robot')
                time.sleep(self.time_step)
        return travel_dist
    # def robot_object_move(self, start, object_num, goal, goal_num):
    #     if goal == 'hTray':
    #         self.sim_env.move_object(object_num, destination_name=goal, destination_num=goal_num)
    #         self.sim_env.root.update_idletasks()
    #     else:
    #         s = np.array(self.rpoints[start])
    #         g = np.array(self.rpoints[goal])
    #         name = start + goal if start + goal in self.rob_slopdist else goal + start
    #         t = round(self.rob_slopdist[name][0] / self.speed)
    #         x = np.linspace(s[0], g[0], round(1 / self.time_step) * t)
    #         y = self.rob_slopdist[name][1] * (x - s[0]) + s[1]
    #         xcur = s[0]
    #         ycur = s[1]
    #         self.sim_env.move_object(object_num, goal=[xcur, ycur])
    #         self.sim_env.root.update_idletasks()
    #
    #         count = 1
    #         while abs(xcur - g[0]) > 0.0:
    #             xcur = x[count]
    #             ycur = y[count]
    #             self.sim_env.move_human_robot([xcur, ycur], 'robot')
    #             self.sim_env.move_object(object_num, goal=[xcur, ycur])
    #             time.sleep(self.time_step)
    #             count += 1
    #         self.sim_env.move_object(object_num, destination_name=goal, destination_num=goal_num)
    #         self.sim_env.root.update_idletasks()

    def robot_object_move_apf(self, start, object_num, goal, goal_num=None, color=None):
        travel_dist = 0
        if goal == 'hTray':
            self.sim_env.move_object(object_num=object_num, destination_name=goal, destination_num=goal_num)
            self.sim_env.root.update_idletasks()

        elif start == 'rTray':
            ll = self.sim_env.table_blocks[color]['status']
            ii = ll.index(0)
            goal_pos = self.sim_env.table_blocks[color]['pos'][ii]
            self.sim_env.move_object(object_num=object_num, goal=goal_pos)
        else:
            s = np.array(self.rpoints[start])
            g = np.array(self.rpoints[goal])
            name = start + goal if start + goal in self.rob_slopdist else goal + start
            t = round(self.rob_slopdist[name][0] / self.speed)
            x = np.linspace(s[0], g[0], round(1 / self.time_step) * t)
            y = self.rob_slopdist[name][1] * (x - s[0]) + s[1]
            xcur = s[0]
            ycur = s[1]
            self.sim_env.move_object(object_num, goal=[xcur, ycur])
            self.sim_env.root.update_idletasks()

            count = 1
            hum_pre_pos_x = None
            hum_pre_pos_y = None
            rd = 50
            while abs(xcur - g[0]) > 10:
                drh2 = (xcur - self.sim_env.human_pos[0]) ** 2 + (ycur - self.sim_env.human_pos[1]) ** 2
                drh = math.sqrt(drh2) - 50

                alpha = math.atan2(g[1] - ycur, g[0] - xcur)
                xatt = math.cos(alpha)
                yatt = math.sin(alpha)
                if drh < self.safe_dist_hr:

                    c1 = (1 / drh - 1 / self.safe_dist_hr) / drh ** 3
                    c2 = 2 * 10 ** 4
                    dxrep = (xcur - self.sim_env.human_pos[0])
                    dyrep = (ycur - self.sim_env.human_pos[1])

                    beta = math.atan2(dyrep, dxrep)
                    xrep = c1 * c2 * math.cos(beta)
                    yrep = c1 * c2 * math.sin(beta)

                    if hum_pre_pos_x is not None:
                        dummy_obs_ang = math.atan2(self.sim_env.human_pos[1] - hum_pre_pos_y,
                                                   self.sim_env.human_pos[0] - hum_pre_pos_x)
                        dummy_x = self.sim_env.human_pos[0] + (25 + rd) * math.cos(dummy_obs_ang)
                        dummy_y = self.sim_env.human_pos[1] + (25 + rd) * math.sin(dummy_obs_ang)
                        # self.sim_env.canvas.create_oval(dummy_x, dummy_y, dummy_x + 2 * rd, dummy_y + 2 * rd)
                        drd2 = (dummy_x - xcur) ** 2 + (dummy_y - ycur) ** 2
                        drd = math.sqrt(drd2) - rd - 25
                        cd1 = (1 / drd - 1 / self.safe_dist_hr) / drd ** 3
                        cd2 = 5 * 10 ** 7
                        ddxrep = (xcur - dummy_x)
                        ddyrep = (ycur - dummy_y)
                        dbeta = math.atan2(ddyrep, ddxrep)
                        xrepd = cd1 * cd2 * math.cos(dbeta)
                        yrepd = cd1 * cd2 * math.sin(dbeta)
                        xrep += xrepd
                        yrep += yrepd
                else:
                    xrep = 0
                    yrep = 0

                speedx = np.sign(xatt + xrep) * min(250, abs(xatt + xrep) * self.speed)
                speedy = np.sign(yatt + yrep) * min(250, abs(yatt + yrep) * self.speed)
                dx = speedx * self.time_step
                dy = speedy * self.time_step
                xcur += dx
                ycur += dy
                travel_dist += math.sqrt(dx**2 + dy**2)
                hum_pre_pos_x = self.sim_env.human_pos[0]
                hum_pre_pos_y = self.sim_env.human_pos[1]

                self.sim_env.move_human_robot([xcur, ycur], 'robot')
                self.sim_env.move_object(object_num, goal=[xcur, ycur])
                time.sleep(self.time_step)
                count += 1
            if goal_num is None:
                ll = self.sim_env.table_blocks[color]['status']
                ii = ll.index(0)
                goal_pos = self.sim_env.table_blocks[color]['pos'][ii]
                self.sim_env.move_object(object_num=object_num, goal=goal_pos)
            else:
                self.sim_env.move_object(object_num, destination_name=goal, destination_num=goal_num)
            self.sim_env.root.update_idletasks()
        return travel_dist

    def robot_action(self, next_action):
        trd1 = 0
        trd2 = 0
        start = next_action['start']
        destination = next_action['destination']
        destination_num = next_action['destination_num']
        object_num = next_action['object']

        if next_action['type'] == 'error1' or next_action['type'] == 'error2':
            self.human.human_wrong_actions.pop(next_action['correcting_action'])
            if next_action['type'] == 'error1':
                trd2 = self.robot_move_apf(start, destination)

            trd1 = self.robot_object_move_apf(start=destination, object_num=object_num, goal=start, color=next_action['color'])
            self.task.available_color_table[next_action['color']].append(object_num)
            ll = self.sim_env.table_blocks[next_action['color']]['status']
            ito = ll.index(0)
            self.sim_env.table_blocks[next_action['color']]['status'][ito] = 1
        elif next_action['type'] == 'tray1' or next_action['type'] == 'tray2':
            trd1 = self.robot_object_move_apf(start=start, object_num=object_num, goal=destination, goal_num=destination_num)
            trd2 = self.robot_move_apf(destination, start)
        else:
            self.task.available_color_table[next_action['color']].pop()
            ll = self.sim_env.table_blocks[next_action['color']]['status']
            ito = len(ll) - 1 - ll[::-1].index(1)
            self.sim_env.table_blocks[next_action['color']]['status'][ito] = 0
            trd1 = self.robot_object_move_apf(start=start, object_num=object_num, goal=destination, goal_num=destination_num)

            if next_action['type'] == 'normal':
                trd2 = self.robot_move_apf(destination, start)

        return trd1+trd2

    def is_task_selection(self, new_rob_task, new_hum_task):
        istasksel = True
        if new_rob_task is None:
            istasksel = True
        elif any(item in new_rob_task for item in self.human.done_tasks):
            istasksel = True

        if self.is_belief_change():
            istasksel = True

        return istasksel

    def is_scheduling(self):
        return True

    def is_belief_change(self):
        return False

    def action_from_schedule(self, timerob, available_actions, precedence, count):
        act_info = None
        if available_actions:

            ac = available_actions[0]
            in_table_zone = False
            if ac in self.task.remained_tasks:
                if ac in self.task.human_error_tasks:
                    if self.task.task_to_do[ac][0] == 'rTray':
                        dest = self.task.task_to_do[ac][0]
                        self.task.human_error_tasks_type2.remove(ac)
                        self.task.available_color_robot_tray[self.task.task_to_do[ac][1]] = []
                        in_table_zone = True
                        etype = 'error2'
                    else:
                        dest = 'W{}'.format(self.task.task_to_do[ac][0])
                        self.task.human_error_tasks_type1.remove(ac)
                        etype = 'error1'
                    act_info = {'type': etype, 'start': 'T', 'destination': dest,
                                'destination_num': self.task.task_to_do[ac][1],
                                'object': self.task.task_to_do[ac][3], 'color': self.task.task_to_do[ac][2],
                                'correcting_action': self.task.task_to_do[ac][4], 'action_number': self.task.task_to_do[ac][4]}
                    self.task.finished_tasks.append(ac)
                    self.task.human_error_tasks.remove(ac)
                else:
                    col = self.task.task_to_do[ac][2]
                    if ac in self.task.tasks_allocated_to_robot:
                        ds = self.task.task_to_do[ac][1]
                        ws = self.task.task_to_do[ac][0]
                        self.task.tasks_allocated_to_robot.remove(ac)
                        act_info = {'type': 'tray1', 'start': 'T', 'destination': 'W{}'.format(ws),
                                    'destination_num': ds,
                                    'object': self.task.available_color_robot_tray[ws], 'action_number': ac, 'color': col}
                        self.task.available_color_robot_tray[ws] = []
                    elif ac in self.task.tasks_allocated_to_human:
                        ds = self.task.task_to_do[ac][1]
                        ws = self.task.task_to_do[ac][0]
                        self.task.tasks_allocated_to_human.remove(ac)
                        act_info = {'type': 'tray2', 'start': 'T', 'destination': 'W{}'.format(ws),
                                    'destination_num': ds,
                                    'object': self.task.available_color_human_tray[ws], 'action_number': ac, 'color': col}
                    else:
                        act_info = {'type': 'normal', 'start': 'T',
                                    'destination': 'W{}'.format(self.task.task_to_do[ac][0]),
                                    'destination_num': self.task.task_to_do[ac][1], 'color': col,
                                    'object': self.task.available_color_table[col][-1], 'action_number': ac}
                    self.task.finished_tasks.append(ac)
            else:
                for i in precedence:
                    if ac in precedence[i]:
                        ds = self.task.task_to_do[i][0]
                        col = self.task.task_to_do[i][2]
                        self.task.tasks_allocated_to_human.append(i)
                        self.all_allocated_tasks.append(i)
                        self.cur_allocated_tasks = self.task.tasks_allocated_to_human[:]
                        break
                act_info = {'type': 'allocate', 'start': 'T', 'destination': 'hTray',
                            'destination_num': ds,
                            'object': self.task.available_color_table[col][-1], 'color': col, 'action_number': i}
                self.task.available_color_human_tray[ds] = self.task.available_color_table[col][-1]
                in_table_zone = True

            # self.task.finished_tasks.append(i)
            available_actions.pop(0)
        else:
            cccccccccccccccccc = 1
        return act_info, in_table_zone, available_actions

    def run(self):
        self.measure.human_measures(self.measure.init_time, self.p_human_allocation, self.p_human_error)
        htmax = max(v[0] for v in list(self.task.t_task_all.values()))
        rtmax = max(v[1] for v in list(self.task.t_task_all.values()))
        punish_h = 1.5 * (rtmax + htmax)
        punish_r = punish_h
        punish_error = 2 * punish_h
        if self.save_init_sol:
            new_human_task, new_robot_task = self.planner.task_selection(task=self.task, hpenalty=punish_h,
                                                                         rpenalty=punish_r, error_penalty=punish_error,
                                                                         save=self.save_init_sol)
            htasks, rtasks, new_pr, new_pr_type2, ttasks = self.task.create_new_task(new_robot_tasks=new_robot_task,
                                                                                     new_human_tasks=new_human_task)
            self.planner.task_scheduler(ttasks, htasks, rtasks, new_pr, new_pr_type2, self.task.remained_tasks,
                                        save=self.save_init_sol)
        counter = 0
        new_robot_task = None
        new_human_task = None
        next_robot_turn = False
        while len(self.task.remained_task_both) + len(self.task.remained_task_robot_only) > 0:
            print(self.task.remained_task_both)
            start_time_total = self.measure.start_time()
            self.task.find_remained_task()
            self.task.remove_finished_task_precedence()

            hum_new_actions = []
            pre_tasks = self.pre_human_tasks_done[:]
            for i in self.human.done_tasks:
                if i in pre_tasks:
                    pre_tasks.remove(i)
                else:
                    hum_new_actions.append(i)
            # hum_new_actions = list(set(self.human.done_tasks) - set(self.pre_human_tasks_done))
            if hum_new_actions:
                if self.cur_allocated_tasks or self.task.tasks_allocated_to_robot:  # Todo: check why I added self.cur_allocated_task
                    for ts in hum_new_actions:
                        if ts in self.cur_allocated_tasks:
                            haction = 1
                        elif ts in self.task.tasks_allocated_to_robot:
                            haction = 0  # Todo: this reduces p_conform significantly
                        else:
                            haction = 0

                        self.planner.adaptability_update(human_action=haction, action_history=self.interaction_history)
                        self.interaction_history.append(haction)
                self.cur_allocated_tasks = self.task.tasks_allocated_to_human[:]

                human_wrong_actions = []
                for ts in hum_new_actions:
                    if ts in self.human.human_wrong_actions:
                        heaction = 0
                        human_wrong_actions.append(ts)
                    elif ts not in self.human.human_actions_from_allocated:
                        heaction = 1

                    if ts not in self.human.human_actions_from_allocated:
                        self.planner.human_error_update(human_action=heaction,
                                                        action_history=self.human_accuracy_history)
                        self.human_accuracy_history.append(heaction)

                self.measure.human_measures(start_time=start_time_total, p_error=self.planner.p_human_error,
                                            p_following=self.planner.p_human_allocation)
                if human_wrong_actions:
                    seen = set()
                    dubl = []
                    for x in human_wrong_actions:
                        if x in seen:
                            dubl.append(x)
                        else:
                            seen.add(x)
                    self.human.double_error = list(set(self.human.double_error) - set(dubl))
                    human_wrong_actions = list(set(human_wrong_actions))
                    de = self.task.update_task_human_error(human_error=human_wrong_actions,
                                                           double_error=self.human.double_error,
                                                           all_human_error=self.human.human_wrong_actions,
                                                           error_info=self.human.wrong_action_info)
                    # self.human.double_error = de[:]

            if next_robot_turn:
                fselec = False
                fschedule = False
            else:
                fselec = self.is_task_selection(new_robot_task, new_human_task)
                if fselec:
                    fschedule = True
                else:
                    fschedule = self.is_scheduling()

            # if fselec:
            #     new_human_task, new_robot_task = self.planner.task_selection(task=self.task, hpenalty=punish_h,
            #                                                                  rpenalty=punish_r,
            #                                                                  error_penalty=punish_error)
            if fschedule:
                is_solution = False
                while not is_solution:
                    if fselec:
                        new_human_task, new_robot_task = self.planner.task_selection(task=self.task,
                                                                                     hpenalty=punish_h,
                                                                                     rpenalty=punish_r,
                                                                                     error_penalty=punish_error,
                                                                                     prev_sol={})

                    htasks, rtasks, new_pr, new_pr_type2, ttasks = self.task.create_new_task(
                        new_robot_tasks=new_robot_task,
                        new_human_tasks=new_human_task)
                    rtiming, htiming, precedence, is_solution = self.planner.task_scheduler(task_time=ttasks,
                                                                                            human_tasks=htasks,
                                                                                            robot_tasks=rtasks,
                                                                                            precedence=new_pr,
                                                                                            precedence_type2=new_pr_type2,
                                                                                            remaining_tasks=self.task.remained_tasks,
                                                                                            tasks_human_error=self.task.human_error_tasks,
                                                                                            tasks_human_error_type1=self.task.human_error_tasks_type1,
                                                                                            tasks_human_error_type2=self.task.human_error_tasks_type2,
                                                                                            )
                cur_step_actions = [i for i in rtiming if rtiming[i] == 0]
                t_tray = [i for i in cur_step_actions if i in new_pr_type2]
                tray_t = [i for i in cur_step_actions if i in self.task.human_error_tasks_type2]
                w_tray = list(set(cur_step_actions) - set(tray_t) - set(t_tray))
                available_actions = tray_t + t_tray + w_tray
                if not available_actions:
                    ccccccccccccc = 1
                counter = 0
            else:
                counter += 1

            next_action, next_robot_turn, available_actions = self.action_from_schedule(timerob=rtiming,
                                                                                        available_actions=available_actions,
                                                                                        precedence=precedence,
                                                                                        count=counter)
            self.pre_human_tasks_done = self.human.done_tasks[:]
            self.pre_human_wrong_actions = list(self.human.human_wrong_actions.keys())
            start_time_action = self.measure.start_time()
            travel_dist = self.robot_action(next_action)
            self.measure.action_end(start_time_total=start_time_total, start_time_action=start_time_action,
                                    agent='robot', travel_distance=travel_dist, action_type=next_action['type'],
                                    action_number=next_action['action_number'])

            aaaaaaa = 1
