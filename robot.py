import math
import numpy as np
import time
import threading
import planner
import copy


class Robot(threading.Thread):
    speed = 1.0

    def __init__(self, task, speed, human, sim_env, time_step):
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
                                       self.rob_slopdist['TW1'][0] * 2 / self.speed)
        for i in range(5, 10):
            self.task.t_task_all[i] = (self.hum_slopdist['TW2'][0] * 2 / self.human.speed,
                                       self.rob_slopdist['TW2'][0] * 2 / self.speed)
        for i in range(10, 15):
            self.task.t_task_all[i] = (self.hum_slopdist['TW3'][0] * 2 / self.human.speed,
                                       self.rob_slopdist['TW3'][0] * 2 / self.speed)
        for i in range(15, 20):
            self.task.t_task_all[i] = (self.hum_slopdist['TW1'][0] * 2 / self.human.speed,
                                       self.rob_slopdist['TW1'][0] * 2 / self.speed)

    def action_selection(self):
        pass

    def robot_move(self, start, goal):
        s = np.array(self.rpoints[start])
        g = np.array(self.rpoints[goal])
        name = start + goal if start + goal in self.rob_slopdist else goal + start
        t = round(self.rob_slopdist[name][0] / self.speed)
        # print(t)
        x = np.linspace(s[0], g[0], round(1 / self.time_step) * t)
        y = self.rob_slopdist[name][1] * (x - s[0]) + s[1]
        xcur = s[0]
        count = 1
        if not (self.sim_env.human_pos[0] < self.sim_env.table_h + 400) or not (xcur < self.sim_env.table_h + 400) \
                or not (xcur > self.sim_env.human_pos[0]):

            while abs(xcur - g[0]) > 0.0:
                xcur = x[count]
                ycur = y[count]
                self.sim_env.move_human_robot([xcur, ycur], 'robot')
                time.sleep(self.time_step)
                count += 1

    def robot_move_apf(self, start, goal):
        s = np.array(self.rpoints[start])
        g = np.array(self.rpoints[goal])
        name = start + goal if start + goal in self.rob_slopdist else goal + start
        t = round(self.rob_slopdist[name][0] / self.speed)
        # print(t)
        x = np.linspace(s[0], g[0], round(1 / self.time_step) * t)
        y = self.rob_slopdist[name][1] * (x - s[0]) + s[1]
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
                        # print('xrep = {:f}, yrep = {:f}, xrepd = {:f}, yrepd ={:f}'.format(xrep, yrep, xrepd, yrepd))
                        xrep += xrepd
                        yrep += yrepd

                        # xrep = 0
                        # yrep = 0
                        # xatt = 0
                        # yatt = 0

                else:
                    xrep = 0
                    yrep = 0

                speedx = min(250, (xatt + xrep) * self.speed)
                speedy = min(250, (yatt + yrep) * self.speed)
                xcur += speedx * self.time_step
                ycur += speedy * self.time_step

                # xcur += (xatt + xrep) * self.speed * self.time_step
                # ycur += (yatt + yrep) * self.speed * self.time_step
                hum_pre_pos_x = self.sim_env.human_pos[0]
                hum_pre_pos_y = self.sim_env.human_pos[1]
                self.sim_env.move_human_robot([xcur, ycur], 'robot')
                time.sleep(self.time_step)

    def robot_object_move(self, start, object_num, goal, goal_num):
        if goal == 'hTray':
            self.sim_env.move_object(object_num, destination_name=goal, destination_num=goal_num)
            self.sim_env.root.update_idletasks()
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
            while abs(xcur - g[0]) > 0.0:
                xcur = x[count]
                ycur = y[count]
                self.sim_env.move_human_robot([xcur, ycur], 'robot')
                self.sim_env.move_object(object_num, goal=[xcur, ycur])
                time.sleep(self.time_step)
                count += 1
            self.sim_env.move_object(object_num, destination_name=goal, destination_num=goal_num)
            self.sim_env.root.update_idletasks()

    def robot_object_move_apf(self, start, object_num, goal, goal_num=None, color=None):
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
                        # print(
                        #     'xrep1 = {:f}, yrep1 = {:f}, xrepd1 = {:f}, yrepd1 ={:f}'.format(xrep, yrep, xrepd, yrepd))
                        xrep += xrepd
                        yrep += yrepd
                        # xrep = 0
                        # yrep = 0
                        # xatt = 0
                        # yatt = 0

                else:
                    xrep = 0
                    yrep = 0

                speedx = min(250, (xatt + xrep) * self.speed)
                speedy = min(250, (yatt + yrep) * self.speed)
                xcur += speedx * self.time_step
                ycur += speedy * self.time_step
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

    def robot_action(self, next_action):
        start = next_action['start']
        destination = next_action['destination']
        destination_num = next_action['destination_num']
        object_num = next_action['object']

        if next_action['type'] == 'error':
            if destination != 'rTray':
                self.robot_move_apf(start, destination)

            self.human.human_wrong_actions.remove(next_action['correcting_action'])
            self.robot_object_move_apf(start=destination, object_num=object_num, goal=start, color=next_action['color'])
            self.task.available_color_table[next_action['color']].append(object_num)
            ll = self.sim_env.table_blocks[next_action['color']]['status']
            ito = ll.index(0)
            self.sim_env.table_blocks[next_action['color']]['status'][ito] = 1
        elif next_action['type'] == 'tray':
            self.robot_object_move_apf(start=start, object_num=object_num, goal=destination, goal_num=destination_num)
            self.robot_move_apf(destination, start)
        else:
            self.task.available_color_table[next_action['color']].pop()
            ll = self.sim_env.table_blocks[next_action['color']]['status']
            ito = len(ll) - 1 - ll[::-1].index(1)
            self.sim_env.table_blocks[next_action['color']]['status'][ito] = 0
            self.robot_object_move_apf(start=start, object_num=object_num, goal=destination, goal_num=destination_num)

            if destination != 'hTray':
                self.robot_move_apf(destination, start)

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
        if available_actions:
            if len(available_actions) > 1:
                if available_actions[0] in self.task.remained_tasks:
                    available_actions.append(available_actions.pop(0))

            ac = available_actions[0]
            twait = 0
            in_table_zone = False
            if ac in self.task.remained_tasks:
                if ac in self.task.human_error_tasks:
                    if self.task.task_to_do[ac][0] == 'rTray':
                        dest = self.task.task_to_do[ac][0]
                        in_table_zone = True
                    else:
                        dest = 'W{}'.format(self.task.task_to_do[ac][0])
                    act_info = {'type': 'error', 'start': 'T', 'destination': dest,
                                'destination_num': self.task.task_to_do[ac][1],
                                'object': self.task.task_to_do[ac][3], 'color': self.task.task_to_do[ac][2],
                                'correcting_action': self.task.task_to_do[ac][4]}
                    self.task.finished_tasks.append(ac)
                else:
                    col = self.task.task_to_do[ac][2]
                    if ac in self.task.tasks_allocated_to_robot:
                        ds = self.task.task_to_do[ac][1]
                        ws = self.task.task_to_do[ac][0]
                        self.task.tasks_allocated_to_robot.remove(ac)
                        act_info = {'type': 'tray', 'start': 'T', 'destination': 'W{}'.format(ws),
                                    'destination_num': ds,
                                    'object': self.task.available_color_robot_tray[ws], 'wait_time': 0, 'color': col}
                    elif ac in self.task.tasks_allocated_to_human:
                        ds = self.task.task_to_do[ac][1]
                        ws = self.task.task_to_do[ac][0]
                        self.task.tasks_allocated_to_human.remove(ac)
                        act_info = {'type': 'tray', 'start': 'T', 'destination': 'W{}'.format(ws),
                                    'destination_num': ds,
                                    'object': self.task.available_color_human_tray[ws], 'wait_time': 0, 'color': col}
                    else:
                        act_info = {'type': 'normal', 'start': 'T',
                                    'destination': 'W{}'.format(self.task.task_to_do[ac][0]),
                                    'destination_num': self.task.task_to_do[ac][1], 'color': col,
                                    'object': self.task.available_color_table[col][-1], 'wait_time': twait}
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
                act_info = {'type': 'normal', 'start': 'T', 'destination': 'hTray',
                            'destination_num': ds,
                            'object': self.task.available_color_table[col][-1], 'color': col, 'wait_time': twait}
                self.task.available_color_human_tray[ds] = self.task.available_color_table[col][-1]
                in_table_zone = True

            # self.task.finished_tasks.append(i)
            available_actions.pop(0)
        return act_info, in_table_zone, available_actions

    def run(self):
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
                    else:
                        heaction = 1

                    self.planner.human_error_update(human_action=heaction, action_history=self.human_accuracy_history)
                    self.human_accuracy_history.append(heaction)

                # hum_new_errors = list(set(self.human.human_wrong_actions) - set(self.pre_human_wrong_actions))
                if human_wrong_actions:
                    self.task.update_task_human_error(human_error=human_wrong_actions,
                                                      error_info=self.human.wrong_action_info)

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
                counter = 0
            else:
                counter += 1

            next_action, next_robot_turn, cur_step_actions = self.action_from_schedule(timerob=rtiming,
                                                                                       available_actions=cur_step_actions,
                                                                                       precedence=precedence,
                                                                                       count=counter)

            # if next_action['destination'] == 'Tray':
            #     self.cur_human_tasks_done = self.human.done_tasks

            # self.robot_action('T', 'W1', 1, 1)
            self.pre_human_tasks_done = self.human.done_tasks[:]
            self.pre_human_wrong_actions = self.human.human_wrong_actions[:]
            self.robot_action(next_action)
