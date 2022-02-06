import numpy as np
import time
import threading
import random


class Human(threading.Thread):
    speed = 1.0

    def __init__(self, task, speed, sim_env, time_step, p_conformity=1, p_error=0, ):
        threading.Thread.__init__(self)
        self.time_step = time_step
        self.task = task
        self.sim_env = sim_env
        self.done_tasks = []
        self.human_wrong_actions = []
        self.wrong_color_object = {}
        self.p_conformity = p_conformity
        self.p_error = p_error
        self.hpoints = sim_env.hpoints
        self.speed = speed
        self.wrong_action_info = {}
        self.slopdist = {}
        self.slop_distance()

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
        while abs(xcur - g[0]) > 0.0:
            if not (self.sim_env.robot_pos[0] < self.sim_env.table_h + 100) or not (xcur < self.sim_env.table_h + 100) \
                    or not (xcur > self.sim_env.robot_pos[0]):
                xcur = x[count]
                ycur = y[count]
                self.sim_env.move_human_robot([xcur, ycur], 'human')
                time.sleep(self.time_step)
                count += 1

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
        while abs(xcur - goal[0]) > 0.0:
            if not (self.sim_env.robot_pos[0] < self.sim_env.table_h + 100) or not (xcur < self.sim_env.table_h + 100):
                xcur = x[count]
                ycur = y[count]
                self.sim_env.move_human_robot([xcur, ycur], 'human')
                time.sleep(self.time_step)
                count += 1

    def human_object_move(self, start, object_num, goal, goal_num):

        if goal == 'rTray':
            self.sim_env.move_object(object_num, destination_name=goal, destination_num=goal_num)
            self.sim_env.root.update_idletasks()
        else:
            s = np.array(self.hpoints[start])
            g = np.array(self.hpoints[goal])
            name = start + goal if start + goal in self.slopdist else goal + start
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

    def human_action(self, next_action):

        start = next_action['start']
        destination = next_action['destination']
        destination_num = next_action['destination_num']
        object_num = next_action['object']
        # wait_time = next_action['wait_time']
        # if wait_time > 0:
        #     time.sleep(wait_time)
        # else:

        self.human_object_move(start, object_num, destination, destination_num)

        if destination != 'rTray':
            self.human_move(destination, start)

    def get_available_tasks(self):
        human_available_task = []
        for i in self.task.remained_tasks:
            robtas = i in self.task.remained_task_robot_only
            preced_check = any(j in self.task.remained_tasks for j in self.task.task_precedence_dict[i])
            rob_alloc = i in self.task.tasks_allocated_to_robot
            wrong_act = i in self.human_wrong_actions

            if (not robtas) and (not preced_check) and (not rob_alloc) and (not wrong_act):
                human_available_task.append(i)

        not_allocated_tasks = list(set(human_available_task) - set(self.task.tasks_allocated_to_human))
        # not_allocated_tasks = list(set(not_allocated_tasks) - set(self.human_wrong_actions))

        tasks_to_allocate = list(set(not_allocated_tasks) - set(self.task.tasks_allocated_to_robot))

        return not_allocated_tasks, tasks_to_allocate

    def action_selection(self):
        not_allocated_tasks, tasks_to_allocate = self.get_available_tasks()

        pf = 1  # random.random()
        wrong_action = False
        alloc_human = False
        if self.task.tasks_allocated_to_human and pf < self.p_conformity:
            next_action = self.task.tasks_allocated_to_human[0]
            self.task.tasks_allocated_to_human.pop(0)
            ds = self.task.task_to_do[next_action][1]
            ws = self.task.task_to_do[next_action][0]
            act_info = {'start': 'T', 'destination': 'W{}'.format(ws),
                        'destination_num': ds,
                        'object': self.task.available_color_human_tray[ws], 'wait_time': 0}

        elif not_allocated_tasks:
            next_action = random.choice(not_allocated_tasks)
            col = self.task.task_to_do[next_action][2]
            if next_action in tasks_to_allocate and (random.random() < 1.4 or self.p_conformity < 1.3):
                ws = self.task.task_to_do[next_action][0]
                act_info = {'start': 'T', 'destination': 'rTray',
                            'destination_num': ws,
                            'object': self.task.available_color_table[col][-1], 'wait_time': 0}
                self.task.available_color_robot_tray[ws] = self.task.available_color_table[col][-1]
                self.task.available_color_table[col].pop()
                ll = self.sim_env.table_blocks[col]['status']
                ito = len(ll) - 1 - ll[::-1].index(1)
                self.sim_env.table_blocks[col]['status'][ito] = 0
                self.task.tasks_allocated_to_robot.append(next_action)
                alloc_human = True

            else:
                if 0 < self.p_error:  # random.random()
                    colp = list(set(['r', 'g', 'b', 'y']) - set(list(col)))
                    wrong_col = random.choice(colp)
                    col = wrong_col
                    wrong_action = True

                act_info = {'start': 'T', 'destination': 'W{}'.format(self.task.task_to_do[next_action][0]),
                            'destination_num': self.task.task_to_do[next_action][1],
                            'object': self.task.available_color_table[col][-1], 'wait_time': 0}
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
            act_info = {'start': 'T', 'destination': 'W{}'.format(ws),
                        'destination_num': ds,
                        'object': self.task.available_color_human_tray[ws], 'wait_time': 0}

        if wrong_action:
            self.human_wrong_actions.append(next_action)
            self.wrong_action_info[next_action] = {'color': col, 'object_num': act_info['object'],
                                                   'workspace': act_info['destination'],
                                                   'position_num': act_info['destination_num']}
        elif alloc_human:
            pass
        else:
            self.task.finished_tasks.append(next_action)
        self.done_tasks.append(next_action)

        return act_info, next_action

    def run(self):
        # self.human_action('T', 'W4', 4, 10)
        # self.human_action('T', 'W3',2,2)
        first_move = True
        while len(self.task.remained_task_both) + len(self.task.remained_task_robot_only) > 0:
            if first_move:
                self.human_move_by_position(self.sim_env.human_pos, self.sim_env.human_pos_table)
                first_move = False
            else:
                self.task.find_remained_task()
                self.task.remove_finished_task_precedence()
                action, action_num = self.action_selection()
                # print(self.task.tasks_allocated_to_human)
                self.human_action(action)

            # if action_num >= 0:
            #     self.task.finished_tasks.append(action_num)
            #     self.done_tasks.append(action_num)
            #     self.task.find_remained_task()
            #     self.task.remove_finished_task_precedence()
