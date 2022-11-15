import casadi as ca
import numpy as np
import time
import matplotlib.pyplot as plt
import os
import yaml
from math import pi


class AckermannMPC():
    def __init__(self, settings_yaml):

        assert os.path.isfile(settings_yaml), settings_yaml
        with open(settings_yaml, 'r') as stream:
            settings = yaml.safe_load(stream)
            self.l_wh = settings['L']  # [m]
            self.acc_max = settings['max_acc']  # [m/s^2]
            self.v_max = settings['max_vel'] * 0.9  # [m/s]
            self.omega_acc_max = settings['max_ang_acc'] * pi / 180  # to radian
            self.omega_max = settings['max_ang_vel'] * pi / 180  # to radian
            self.with_chair = settings['start_with_chair']
            self.T = settings["mpc_dt"]  # sampling time [s]
            self.N = settings["pred_hrzn"]  # prediction horizon

        self.first_iter = True
        self.start_time = time.time()
        self.states_for_visualize = []
        self.index_t = []
        self.is_stop = False
        self.control_time = np.array([self.T * i for i in range(self.N)])

        self.create_mpc_model()

    def create_mpc_model(self):
        # states
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        theta = ca.SX.sym('theta')
        states = ca.vertcat(x, y, theta)
        self.n_states = states.size()[0]

        # control inputs
        v_x = ca.SX.sym('v_x')
        v_y = ca.SX.sym('v_y')
        v_theta = ca.SX.sym('v_theta')
        controls = ca.vertcat(v_x, v_y, v_theta)
        self.n_controls = controls.size()[0]

        # rhs
        rhs = ca.vertcat(v_x, self.l_wh * v_theta, v_theta)

        # function
        self.f = ca.Function('f', [states, controls], [rhs], ['input_state', 'control_input'], ['rhs'])

        # for MPC
        # control command
        U = ca.SX.sym('U', self.n_controls, self.N)
        # state predictions
        X = ca.SX.sym('X', self.n_states, self.N + 1)
        # parameters with [initial x_y_theta], [goal x_y_theta], [initial x_dot y_dot theta_dot]
        P = ca.SX.sym('P', self.n_states + self.n_states + self.n_controls)

        # define regularization matrix
        Q = np.array([[1.0, 0.0, 0.0], [0.0, 1.5, 0.0], [0.0, 0.0, 1.5]])
        R = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 0.0]])

        # constrains
        g = []
        self.lbg = []  # lower bound of the constraints
        self.ubg = []  # upper bound of the constraints
        self.lbx = []  # decision variables lower bound
        self.ubx = []  # decision variables upper bound

        # initial state
        g.append(X[:, 0] - P[:self.n_states])
        # Initial state constraint
        self.lbg.append(0.0)
        self.lbg.append(0.0)
        self.lbg.append(0.0)
        self.ubg.append(0.0)
        self.ubg.append(0.0)
        self.ubg.append(0.0)

        # cost function
        obj = 0
        for i in range(self.N):
            obj = obj + ca.mtimes([(X[:, i] - P[self.n_states:2 * self.n_states]).T, Q,
                                   X[:, i] - P[self.n_states:2 * self.n_states]]) + ca.mtimes([U[:, i].T, R, U[:, i]])
            x_next_ = self.f(X[:, i], U[:, i]) * self.T + X[:, i]
            # velocity limit
            g.append((U[0, i] - U[1, i]) ** 2)
            self.lbg.append(-self.v_max ** 2)
            self.ubg.append(self.v_max ** 2)
            # Ackermann motion
            g.append(U[2, i] * self.l_wh - U[1, i])
            self.lbg.append(0.0)
            self.ubg.append(0.0)
            # dynamics constraint
            g.append(X[:, i + 1] - x_next_)
            self.lbg.append(0.0)
            self.lbg.append(0.0)
            self.lbg.append(0.0)
            self.ubg.append(0.0)
            self.ubg.append(0.0)
            self.ubg.append(0.0)

        # linear and angular acceleration constraints
        g.append(((U[0, 0] - P[2 * self.n_states]) ** 2 + (U[1, 0] - P[2 * self.n_states + 1]) ** 2) / self.T)
        # self.lbg.append(-self.acc_max**2)
        self.lbg.append(0)
        self.ubg.append(self.acc_max**2)
        g.append((U[2, 0] - P[2 * self.n_states + 2]) / self.T)
        self.lbg.append(-self.omega_acc_max)
        self.ubg.append(self.omega_acc_max)

        for j in range(self.N - 1):
            # linear acceleration
            g.append(((U[0, j + 1] - U[0, j]) ** 2 + (U[1, j + 1] - U[1, j]) ** 2) / self.T)
            self.lbg.append(-self.acc_max)
            # self.lbg.append(0)
            self.ubg.append(self.acc_max**2)

            # angular acceleration
            g.append((U[2, j + 1] - U[2, j]) / self.T)
            self.lbg.append(-self.omega_acc_max)
            self.ubg.append(self.omega_acc_max)

        opt_variables = ca.vertcat(ca.reshape(U, -1, 1), ca.reshape(X, -1, 1))

        nlp_prob = {'f': obj, 'x': opt_variables, 'p': P, 'g': ca.vertcat(*g)}
        opts_setting = {'ipopt.max_iter': 100,
                        'ipopt.print_level': 0,
                        'print_time': 0,
                        'ipopt.acceptable_tol': 1e-8,
                        'ipopt.acceptable_obj_change_tol': 1e-6,
                        'ipopt.gamma_theta': 1e-10}

        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts_setting)

        # Constraint on input(U) [x_vel, y_vel, ang vel]
        for _ in range(self.N):
            self.lbx.append(-self.v_max)
            self.lbx.append(-self.v_max)
            self.lbx.append(-self.omega_max)
            self.ubx.append(self.v_max)
            self.ubx.append(self.v_max)
            self.ubx.append(self.omega_max)

        # Constraint on state(X) [x, y, theta], unconstrained
        for _ in range(self.N + 1):  # note that this is different with the method using structure
            self.lbx.append(-np.inf)
            self.lbx.append(-np.inf)
            self.lbx.append(-np.pi)
            self.ubx.append(np.inf)
            self.ubx.append(np.inf)
            self.ubx.append(np.pi)

        self.u0 = np.array([self.v_max, 0, 0] * self.N)  # input sequence
        self.x_m = np.zeros((self.n_states, self.N + 1))  # traj prediction

    def solve(self, cur_y, cur_theta, cur_x_dot, cur_y_dot, cur_omega):
        # initial state
        x0 = np.array([0, 0, 0]).reshape(-1, 1)  # initial state
        # get goal state
        if cur_theta != 0:
            goal_x = abs(cur_y) / np.sin(abs(cur_theta))
            goal_y = 0
            goal_theta = - cur_theta
            if abs(goal_theta) < (70 * np.pi / 180):
                if goal_x > self.v_max * self.T * self.N:
                    # clipping goal_x
                    goal_x = self.v_max * self.T * self.N
                goal_y = -cur_y * np.cos(abs(cur_theta))
        else:
            goal_x = self.v_max * self.T * self.N
            goal_y = -cur_y * np.cos(abs(cur_theta))
            goal_theta = 0
        xs = np.array([goal_x, goal_y, goal_theta]).reshape(-1, 1)  # final state
        vel0 = np.array([cur_x_dot, cur_y_dot, cur_omega]).reshape(-1, 1)
        # start MPC
        # set parameter
        c_p = np.concatenate((x0, xs))
        c_p = np.concatenate((c_p, vel0))
        self.x_m = np.concatenate((self.x_m[1:], self.x_m[-1:]), axis=0)
        self.u0 = np.concatenate((self.u0[1:], self.u0[-1:]), axis=0)
        init_control = np.concatenate((self.u0.reshape(-1, 1), self.x_m.copy().reshape(-1, 1)))
        # t_ = time.time()
        res = self.solver(x0=init_control, p=c_p, lbg=self.lbg, lbx=self.lbx, ubg=self.ubg, ubx=self.ubx)
        # self.index_t.append(time.time() - t_)
        # the feedback is in the series [u0, x0, u1, x1, ...]
        estimated_opt = res['x'].full()
        self.u0 = estimated_opt[:self.n_controls * self.N].reshape(self.N, self.n_controls)  # (N, n_controls)
        self.x_m = estimated_opt[self.n_controls * self.N:].reshape(self.N + 1, self.n_states)  # (N+1, n_states)

        return self.u0, self.x_m
