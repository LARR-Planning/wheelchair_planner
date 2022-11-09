import casadi as ca
import numpy as np
import time
import os
import yaml
from math import pi, atan2
import rospy
from geometry_msgs.msg import PoseStamped, Vector3, AccelStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class MPCAckermannNode():
    def __init__(self, settings_yaml):
        assert os.path.isfile(settings_yaml), settings_yaml
        with open(settings_yaml, 'r') as stream:
            settings = yaml.safe_load(stream)
            self.l_wh = settings['L']  # [m]
            self.acc_max = settings['max_acc']  # [m/s^2]
            self.v_max = settings['max_vel']  # [m/s]
            self.omega_acc_max = settings['max_ang_acc'] * pi / 180  # to radian
            self.omega_max = settings['max_ang_vel'] * pi / 180  # to radian
            self.with_chair = settings['start_with_chair']
            self.T = settings["npc_dt"]  # sampling time [s]
            self.N = settings["pred_hrzn"]  # prediction horizon

        self.first_iter = True
        self.start_time = time.time()
        self.states_for_visualize = []
        self.index_t = []
        self.is_stop = False
        self.control_time = np.array([self.T * i for i in range(self.N)])

        self.create_mpc_model()

        rospy.init_node('wc_sim_node', anonymous=True)
        # Subscriber
        self.error_sub = rospy.Subscriber("error_from_line", PoseStamped, self.callback_error, queue_size=1)
        # Publisher
        self.command_pub = rospy.Publisher("robot_vel_command", Vector3, queue_size=3)

    def callback_error(self, data):
        error: PoseStamped = data
        error_y = error.pose.position.y
        error_theta = 2 * atan2(error.pose.orientation.z, error.pose.orientation.w)
        # print(f"error y : {error_y}, error_theta : {error_theta}, time : {error.header.stamp}")
        self.state_update(error_y, error_theta, 0, 0)
        vel_command, steering_angle = self.solve()[0]
        x_vel, y_vel, ang_vel = vel_command * np.cos(steering_angle), vel_command * np.sin(
            steering_angle), vel_command * np.sin(steering_angle) / self.l_wh
        command = Vector3()
        command.x = x_vel
        command.y = y_vel
        command.z = ang_vel
        self.command_pub.publish(command)

    def create_mpc_model(self):
        # states
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        theta = ca.SX.sym('theta')
        states = ca.vertcat(x, y)
        states = ca.vertcat(states, theta)
        self.n_states = states.size()[0]

        # control inputs
        v = ca.SX.sym('v')
        steering = ca.SX.sym('steering')
        controls = ca.vertcat(v, steering)
        self.n_controls = controls.size()[0]

        # rhs
        rhs = ca.vertcat(v * ca.cos(steering), v * ca.sin(steering))
        rhs = ca.vertcat(rhs, v * ca.sin(steering) / self.l_wh)

        # function
        self.f = ca.Function('f', [states, controls], [rhs], ['input_state', 'control_input'], ['rhs'])

        # for MPC
        U = ca.SX.sym('U', self.n_controls, self.N)
        X = ca.SX.sym('X', self.n_states, self.N + 1)  # state predictions are stored in this matrix
        P = ca.SX.sym('P',
                      self.n_states + self.n_states)  # parameters which include the initial (in every step) and the reference state of the robot

        # define
        Q = np.array([[1.0, 0.0, 0.0], [0.0, 1.5, 0.0], [0.0, 0.0, 1.5]])
        R = np.array([[1.0, 0.0], [0.0, 0.0]])

        # cost function
        obj = 0  # cost
        g = []  # equal constrains
        g.append(X[:, 0] - P[:3])  # initial state
        for i in range(self.N):
            obj = obj + ca.mtimes([(X[:, i] - P[3:]).T, Q, X[:, i] - P[3:]]) + ca.mtimes([U[:, i].T, R, U[:, i]])
            x_next_ = self.f(X[:, i], U[:, i]) * self.T + X[:, i]
            g.append(X[:, i + 1] - x_next_)
            g.append((X[2, i + 1] - X[2, i]) / self.T)  # angular velocity

        # linear and angular acceleration constraints
        for j in range(self.N - 1):
            g.append((U[0, j + 1] - U[0, j]) / self.T)  # linear acceleration
            g.append(((X[2, j + 2] - X[2, j + 1]) / self.T - (
                    X[2, j + 1] - X[2, j]) / self.T) / self.T)  # angular acceleration

        opt_variables = ca.vertcat(ca.reshape(U, -1, 1), ca.reshape(X, -1, 1))

        nlp_prob = {'f': obj, 'x': opt_variables, 'p': P, 'g': ca.vertcat(*g)}
        opts_setting = {'ipopt.max_iter': 100, 'ipopt.print_level': 0, 'print_time': 0, 'ipopt.acceptable_tol': 1e-8,
                        'ipopt.acceptable_obj_change_tol': 1e-6}

        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts_setting)

        # inequality constraints (state constraints)
        self.lbg = []  # lower bount of the constraints
        self.ubg = []  # upper bount of the constraints
        self.lbx = []  # decision variables lower bound
        self.ubx = []  # decision variables upper bound

        self.lbg.append(0.0)
        self.lbg.append(0.0)
        self.lbg.append(0.0)
        self.ubg.append(0.0)
        self.ubg.append(0.0)
        self.ubg.append(0.0)
        for _ in range(self.N):
            self.lbg.append(0.0)
            self.lbg.append(0.0)
            self.lbg.append(0.0)
            self.lbg.append(-self.omega_max)
            self.ubg.append(0.0)
            self.ubg.append(0.0)
            self.ubg.append(0.0)
            self.ubg.append(self.omega_max)
        for _ in range(self.N - 1):
            self.lbg.append(-self.acc_max)
            self.lbg.append(-self.omega_acc_max)
            self.ubg.append(self.acc_max)
            self.ubg.append(self.omega_acc_max)
        for _ in range(self.N):
            self.lbx.append(0)
            self.lbx.append(-np.pi / 2)
            self.ubx.append(self.v_max)
            self.ubx.append(np.pi / 2)
        for _ in range(self.N + 1):  # note that this is different with the method using structure
            self.lbx.append(-np.inf)
            self.lbx.append(-np.inf)
            self.lbx.append(-np.pi)
            self.ubx.append(np.inf)
            self.ubx.append(np.inf)
            self.ubx.append(np.pi)

    def state_update(self, current_y, current_theta, current_vel, current_omega):
        self.current_y = current_y
        self.current_theta = current_theta
        self.current_vel = current_vel
        if self.current_vel == 0:
            self.current_steering = 0
        else:
            self.current_steering = np.arcsin(current_omega * self.l_wh / self.current_vel)

    def solve(self):
        # initial and goal states
        initial_loc = (0, 0, 0)
        if self.current_theta != 0:
            goal_x, goal_y, goal_theta = self.current_y / np.sin(abs(self.current_theta)), 0, - self.current_theta
            if abs(goal_theta) < (70 * np.pi / 180):
                if goal_x > self.v_max * self.T * self.N:
                    goal_x = self.v_max * self.T * self.N
                goal_y = -self.current_y * np.cos(abs(self.current_theta))
        else:
            goal_x, goal_y, goal_theta = 0, 0, 0
        goal_loc = (goal_x, goal_y, goal_theta)

        t0 = 0.0
        x0 = np.array(initial_loc).reshape(-1, 1)  # initial state
        x_m = np.zeros((self.n_states, self.N + 1))
        next_states = x_m.copy().T
        xs = np.array(goal_loc).reshape(-1, 1)  # final state

        # initial control inputs
        if self.first_iter:
            self.u0 = np.array([self.current_vel, self.current_steering] * self.N).reshape(-1, 2)
        else:
            self.u0[0] = [self.current_vel, self.current_steering]
        self.first_iter = False

        # start MPC
        if np.linalg.norm(x0 - xs) > 1e-2:  # and self.mpciter-sim_time/self.T < 0.0):
            # set parameter
            c_p = np.concatenate((x0, xs))
            init_control = np.concatenate((self.u0.reshape(-1, 1), next_states.reshape(-1, 1)))
            res = self.solver(x0=init_control, p=c_p, lbg=self.lbg, lbx=self.lbx, ubg=self.ubg, ubx=self.ubx)
            # the feedback is in the series [u0, x0, u1, x1, ...]
            estimated_opt = res['x'].full()
            self.u0 = estimated_opt[:self.n_controls * self.N].reshape(self.N, self.n_controls)  # (N, n_controls)
            x_m = estimated_opt[self.n_controls * self.N:].reshape(self.N + 1, self.n_states)  # (N+1, n_states)
            self.control_output = self.u0
            return self.u0
            # Move straight when the robot close enough to the goal line
        else:
            self.u0 = np.array([[self.v_max, 0]] * self.N)
            return self.u0


if __name__ == '__main__':
    mpc_planner = MPCAckermannNode('../simulator/settings.yaml')
    rospy.spin()
