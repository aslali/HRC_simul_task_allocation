import numpy as np
import time
import threading
import random


class Human(threading.Thread):
    speed = 1.0

    def __init__(self, task, speed, sim_env, time_step, measure, p_conformity=1, p_error=0 ):
        threading.Thread.__init__(self)
        self.time_step = time_step
        self.task = task
        self.sim_env = sim_env
        self.done_tasks = []
        self.human_wrong_actions = {}
        self.wrong_color_object = {}
        self.p_conformity = p_conformity
        self.p_error = p_error
        self.hpoints = sim_env.hpoints
        self.speed = speed
        self.wrong_action_info = {}
        self.slopdist = {}
        self.double_error = []
        self.human_actions_from_allocated = []
        self.slop_distance()
        self.measure = measure

        # print(self.rob_slopdist)

    def slop_distance(self):
        cases = ['W1', 'W2', 'W3', 'W4']
        for i in cases:
            s = np.array(self.hpoints['T'])
            g = np.array(self.hpoints[i])
            m = (s[1] - g[1]) / (s[0] - g[0])
            d2 = (s[0] - g[0]) ** 2 + (s[1] - g[1]) ** 2
            d = np.sqrt(d2)
            name = 'T' + i
            self.slopdist[name] = (d, m)
        # print(self.rob_slopdist)

    def human_move(self, start, goal):
        s = np.array(self.hpoints[start])
        g = np.array(self.hpoints[goal])
        name = start + goal if start + goal in self.slopdist else goal + start
        t = round(self.slopdist[name][0] / self.speed)
        # print(t)
        x = np.linspace(s[0], g[0], round(1 / self.time_step) * t)
        y = self.slopdist[name][1] * (x - s[0]) + s[1]
        xcur = s[0]
        count = 1
        timer_started = False
        t1 = 0
        while abs(xcur - g[0]) > 0.0:
            if not (self.sim_env.robot_pos[0] < self.sim_env.table_h + 100) or not (xcur < self.sim_env.table_h + 100) \
                    or not (xcur > self.sim_env.robot_pos[0]):
                xcur = x[count]
                ycur = y[count]
                self.sim_env.move_human_robot([xcur, ycur], 'human')
                time.sleep(self.time_step)
                count += 1
            elif not timer_started:
                t1 = self.measure.start_time()
                timer_started = True

        if t1 == 0:
            t2 = 0
        else:
            t2 = self.measure.start_time()

        return t2 - t1, self.slopdist[name][0]

    def human_move_by_position(self, start, goal):
        m = (start[1] - goal[1]) / (start[0] - goal[0])
        d2 = (start[0] - goal[0]) ** 2 + (start[1] - goal[1]) ** 2
        d = np.sqrt(d2)
        t = int(np.ceil(d / self.speed))
        # print(t)
        x = np.linspace(start[0], goal[0], round(1 / self.time_step) * t)
        y = m * (x - start[0]) + start[1]
        xcur = start[0]
        count = 1
        timer_started = False
        t1 = 0
        while abs(xcur - goal[0]) > 0.0:
            if not (self.sim_env.robot_pos[0] < self.sim_env.table_h + 100) or not (xcur < self.sim_env.table_h + 100):
                xcur = x[count]
                ycur = y[count]
                self.sim_env.move_human_robot([xcur, ycur], 'human')
                time.sleep(self.time_step)
                count += 1
            elif not timer_started:
                t1 = self.measure.start_time()
                timer_started = True

        if t1 == 0:
            t2 = 0
        else:
            t2 = self.measure.start_time()

        return t2-t1, d

    def human_object_move(self, start, object_num, goal, goal_num):
        d1 = 0
        if goal == 'rTray':
            self.sim_env.move_object(object_num, destination_name=goal, destination_num=goal_num)
            self.sim_env.root.update_idletasks()
        else:
            s = np.array(self.hpoints[start])
            g = np.array(self.hpoints[goal])
            name = start + goal if start + goal in self.slopdist else goal + start
            d1 = self.slopdist[name][0]
            t = round(self.slopdist[name][0] / self.speed)
            x = np.linspace(s[0], g[0], round(1 / self.time_step) * t)
            y = self.slopdist[name][1] * (x - s[0]) + s[1]
            xcur = s[0]
            ycur = s[1]
            self.sim_env.move_object(object_num, goal=[xcur, ycur])
            self.sim_env.root.update_idletasks()

            count = 1
            while abs(xcur - g[0]) > 0.0:
                xcur = x[count]
                ycur = y[count]
                self.sim_env.move_human_robot([xcur, ycur], 'human')
                self.sim_env.move_object(object_num, goal=[xcur, ycur])
                time.sleep(self.time_step)
                count += 1
            self.sim_env.move_object(object_num, destination_name=goal, destination_num=goal_num)
            self.sim_env.root.update_idletasks()

        return d1

    def human_action(self, next_action):
        idle_time = 0
        d2 = 0
        start = next_action['start']
        destination = next_action['destination']
        destination_num = next_action['destination_num']
        object_num = next_action['object']
        # wait_time = next_action['wait_time']
        # if wait_time > 0:
        #     time.sleep(wait_time)
        # else:

        d1 = self.human_object_move(start, object_num, destination, destination_num)

        if destination != 'rTray':
            idle_time, d2 = self.human_move(destination, start)
        return idle_time, d1+d2

    def get_available_tasks(self):
        human_available_task = []
        human_available_task_error = []
        human_available_wrong_tasks = []
        type1_error = [i for i in self.human_wrong_actions if self.human_wrong_actions[i] == 'type1']
        # type2_error = [i for i in self.human_wrong_actions if self.human_wrong_actions[i] == 'type2']
        cor_wrong_actions = list(set(self.task.remained_tasks) - set(type1_error))
        for i in self.task.remained_tasks:
            robtas = i in self.task.remained_task_robot_only
            preced_check_with_error = any(j in cor_wrong_actions for j in self.task.task_precedence_dict[i])
            preced_check_no_error = any(j in self.task.remained_tasks for j in self.task.task_precedence_dict[i])
            rob_alloc = i in self.task.tasks_allocated_to_robot
            wrong_act = i in self.human_wrong_actions

            if (not robtas) and (not preced_check_with_error) and (not rob_alloc) and (not wrong_act):
                human_available_task_error.append(i)
                if not preced_check_no_error:
                    human_available_task.append(i)
                else:
                    human_available_wrong_tasks.append(i)

        not_allocated_tasks = list(set(human_available_task) - set(self.task.tasks_allocated_to_human))
        # not_allocated_tasks = list(set(not_allocated_tasks) - set(self.human_wrong_actions))

        tasks_to_allocate = list(set(not_allocated_tasks) - set(self.task.tasks_allocated_to_robot))

        return not_allocated_tasks, tasks_to_allocate, human_available_wrong_tasks

    def action_selection(self):
        not_allocated_tasks, tasks_to_allocate, human_available_wrong_tasks = self.get_available_tasks()

        pf = random.random()
        wrong_action_type1 = False
        wrong_action_type2 = False
        alloc_robot = False
        act_info = {}
        next_action = None
        is_error = random.random() < self.p_error   #random.random()

        if self.task.tasks_allocated_to_human and pf < self.p_conformity:
            next_action = self.task.tasks_allocated_to_human[0]
            self.task.tasks_allocated_to_human.pop(0)
            ds = self.task.task_to_do[next_action][1]
            ws = self.task.task_to_do[next_action][0]
            act_info = {'type':'tray1', 'start': 'T', 'destination': 'W{}'.format(ws),
                        'destination_num': ds,
                        'object': self.task.available_color_human_tray[ws], 'action_number': next_action}
            self.task.available_color_human_tray[ws] = []
            self.human_actions_from_allocated.append(next_action)

        elif not_allocated_tasks or (is_error and human_available_wrong_tasks):
            if is_error:
                next_action = random.choice(not_allocated_tasks + human_available_wrong_tasks)
            else:
                next_action = random.choice(not_allocated_tasks)
            col = self.task.task_to_do[next_action][2]
            cond1 = (next_action in tasks_to_allocate) and (random.random() < 0.2 or self.p_conformity < 0.35)
            cond2 = len(not_allocated_tasks) > 1 or self.task.tasks_allocated_to_human
            if cond1 and cond2:
                ws = self.task.task_to_do[next_action][0]
                if is_error:
                    if next_action in human_available_wrong_tasks:
                        colp = ['r', 'g', 'b', 'y']
                    else:
                        colp = list(set(['r', 'g', 'b', 'y']) - set(list(col)))

                    wrong_col = random.choice(colp)
                    col = wrong_col
                    wrong_action_type2 = True
                    atype = 'error2'
                else:
                    self.task.tasks_allocated_to_robot.append(next_action)
                    alloc_robot = True
                    atype = 'allocate'

                act_info = {'type': atype, 'start': 'T', 'destination': 'rTray',
                            'destination_num': ws,
                            'object': self.task.available_color_table[col][-1], 'action_number': next_action}
                self.task.available_color_robot_tray[ws] = self.task.available_color_table[col][-1]
                self.task.available_color_table[col].pop()
                ll = self.sim_env.table_blocks[col]['status']
                ito = len(ll) - 1 - ll[::-1].index(1)
                self.sim_env.table_blocks[col]['status'][ito] = 0

            else:
                if is_error:
                    if next_action in human_available_wrong_tasks:
                        colp = ['r', 'g', 'b', 'y']
                    else:
                        colp = list(set(['r', 'g', 'b', 'y']) - set(list(col)))
                    wrong_col = random.choice(colp)
                    col = wrong_col
                    wrong_action_type1 = True
                    atype = 'error1'
                else:
                    atype = 'normal'

                act_info = {'type': atype, 'start': 'T', 'destination': 'W{}'.format(self.task.task_to_do[next_action][0]),
                            'destination_num': self.task.task_to_do[next_action][1],
                            'object': self.task.available_color_table[col][-1], 'action_number': next_action}
                self.task.available_color_table[col].pop()
                ll = self.sim_env.table_blocks[col]['status']
                ito = len(ll) - 1 - ll[::-1].index(1)
                self.sim_env.table_blocks[col]['status'][ito] = 0

        elif self.task.tasks_allocated_to_human:
            ac = random.randint(0, len(self.task.tasks_allocated_to_human) - 1)
            next_action = self.task.tasks_allocated_to_human[ac]
            self.task.tasks_allocated_to_human.pop(ac)
            col = self.task.task_to_do[next_action][2]
            ds = self.task.task_to_do[next_action][1]
            ws = self.task.task_to_do[next_action][0]
            act_info = {'type':'tray1', 'start': 'T', 'destination': 'W{}'.format(ws),
                        'destination_num': ds,
                        'object': self.task.available_color_human_tray[ws], 'action_number': next_action}
            self.task.available_color_human_tray[ws] = []
            self.human_actions_from_allocated.append(next_action)
        elif not not_allocated_tasks:
            pa = [i for i in self.human_wrong_actions if self.human_wrong_actions[i] == 'type2']
            pa += self.task.tasks_allocated_to_robot
            if pa:
                next_action = random.choice(pa)
                if next_action in self.human_wrong_actions:
                    ds = self.task.task_to_do[next_action][1]
                    ws = self.task.task_to_do[next_action][0]
                    act_info = {'type': 'error1', 'start': 'T', 'destination': 'W{}'.format(ws),
                                'destination_num': ds,
                                'object': self.task.available_color_robot_tray[ws], 'action_number': next_action}
                    col = self.wrong_action_info[next_action]['color']
                    wrong_action_type1 = True
                    self.double_error.append(next_action)
                else:
                    self.task.tasks_allocated_to_human.remove(next_action)
                    # col = self.task.task_to_do[next_action][2]
                    ds = self.task.task_to_do[next_action][1]
                    ws = self.task.task_to_do[next_action][0]
                    act_info = {'type': 'tray2', 'start': 'T', 'destination': 'W{}'.format(ws),
                                'destination_num': ds,
                                'object': self.task.available_color_robot_tray[ws], 'action_number': next_action}
        else:
            raise ValueError('Unconsidered Case')

        if wrong_action_type1 or wrong_action_type2:
            self.human_wrong_actions[next_action] = 'type1' if wrong_action_type1 else 'type2'
            self.wrong_action_info[next_action] = {'color': col, 'object_num': act_info['object'],
                                                   'workspace': act_info['destination'],
                                                   'position_num': act_info['destination_num']}
        elif alloc_robot:
            pass
        else:
            self.task.finished_tasks.append(next_action)
        if next_action is not None:
            self.done_tasks.append(next_action)

        return act_info, next_action

    def run(self):
        # self.human_action('T', 'W4', 4, 10)
        # self.human_action('T', 'W3',2,2)
        first_move = True

        while len(self.task.remained_task_both) + len(self.task.remained_task_robot_only) > 0:
            idle_time = 0
            travel_dist = 0
            start_time = self.measure.start_time()
            if first_move:
                idle_time, travel_dist = self.human_move_by_position(self.sim_env.human_pos, self.sim_env.human_pos_table)
                first_move = False
                action = None
            else:
                self.task.find_remained_task()
                self.task.remove_finished_task_precedence()
                action, action_num = self.action_selection()
                # print(action, 'num: ', action_num)
                if action:
                    idle_time, travel_dist = self.human_action(action)
            self.measure.action_end(start_time_total=start_time, agent='human', idle_time=idle_time,
                                    travel_distance=travel_dist, action_type=action['type'] if action else 'idle',
                                    action_number=action['action_number'] if action else -1)

