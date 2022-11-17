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
            self.v_max = settings['max_vel'] * 0.99  # [m/s]
            self.omega_acc_max = settings['max_ang_acc'] * pi / 180  # to radian
            self.omega_max = settings['max_ang_vel'] * pi / 180  *0.99  # to radian
            self.with_chair = settings['start_with_chair']
            self.T = settings["mpc_dt"]  # sampling time [s]
            self.N = settings["pred_hrzn"]  # prediction horizon

        self.first_iter = True
        self.start_time = time.time()
        self.states_for_visualize = []
        self.index_t = []
        self.is_stop = False
        self.no_chair = False
        self.control_time = np.array([self.T * i for i in range(self.N + 1)])

        self.create_mpc_model()

    def state_update(self, current_y, current_theta, current_vel, current_omega, is_stop, no_chair):
        self.current_y = current_y
        self.current_theta = current_theta
        self.current_vel = current_vel
        if self.current_vel == 0:
            self.current_steering = 0
        else:
            self.current_steering = np.arcsin(np.clip(current_omega*self.l_wh/self.current_vel,-1,1))
        self.is_callback = True
        self.is_stop = is_stop
        self.no_chair = no_chair

    def create_mpc_model(self):
        # states
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        theta = ca.SX.sym('theta')
        states = ca.vertcat(x,y)
        states = ca.vertcat(states, theta)
        self.n_states = states.size()[0]

        # control inputs
        v = ca.SX.sym('v')
        steering = ca.SX.sym('steering')
        controls = ca.vertcat(v, steering)
        self.n_controls = controls.size()[0]

        # rhs
        rhs = ca.vertcat(v*ca.cos(steering), v*ca.sin(steering))
        rhs = ca.vertcat(rhs, v*ca.sin(steering)/self.l_wh)

        # function
        self.with_chair_f = ca.Function('f', [states, controls], [rhs], ['input_state', 'control_input'], ['rhs'])

        # for MPC
        U = ca.SX.sym('U', self.n_controls, self.N)
        X = ca.SX.sym('X', self.n_states, self.N+1)   # state predictions are stored in this matrix
        P = ca.SX.sym('P', self.n_states + self.n_states)  # parameters which include the initial (in every step) and the reference state of the robot

        # define
        Q = np.array([[1.0, 0.0, 0.0], [0.0, 1.5, 0.0], [0.0, 0.0, 1.5]])
        R = np.array([[1.0, 0.0], [0.0, 0.0]])
        R_is_stop = np.array([[2.0, 0.0], [0.0, 1.0]])

        # cost function
        obj = 0 # cost
        obj_is_stop = 0
        g = []  # equal constrains
        g.append(X[:,0] - P[:3]) # initial state
        for i in range(self.N):
            obj = obj + ca.mtimes([(X[:,i]-P[3:]).T, Q, X[:,i]-P[3:]]) + ca.mtimes([U[:,i].T, R, U[:,i]])
            obj_is_stop = obj_is_stop + ca.mtimes([U[:,i].T, R_is_stop, U[:,i]])
            x_next_ = self.with_chair_f(X[:,i], U[:,i])*self.T + X[:,i]
            g.append(X[:, i+1]-x_next_)
            g.append((X[2, i+1]-X[2, i])/self.T) # angular velocity

        # linear and angular acceleration constraints
        for j in range(self.N-1):
            g.append((U[0,j+1]-U[0,j])/self.T)   # linear acceleration
            g.append(((X[2, j+2]-X[2, j+1])/self.T-(X[2, j+1]-X[2, j])/self.T)/self.T)   # angular acceleration

        opt_variables = ca.vertcat(ca.reshape(U,-1,1), ca.reshape(X,-1,1))

        nlp_prob = {'f': obj, 'x': opt_variables, 'p': P, 'g': ca.vertcat(*g)}
        opts_setting = {'ipopt.max_iter': 100, 'ipopt.print_level': 0, 'print_time': 0, 'ipopt.acceptable_tol': 1e-8, 'ipopt.acceptable_obj_change_tol': 1e-6}

        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts_setting)

        # inequality constraints (state constraints)
        self.lbg = []   # lower bound of the constraints
        self.ubg = []   # upper bound of the constraints
        self.lbx = []   # decision variables lower bound
        self.ubx = []   # decision variables upper bound

        # Initial state constraint
        self.lbg.append(0.0)
        self.lbg.append(0.0)
        self.lbg.append(0.0)
        self.ubg.append(0.0)
        self.ubg.append(0.0)
        self.ubg.append(0.0)

        # Dynamics & Angular Velocity constraint
        for _ in range(self.N):
            self.lbg.append(0.0)
            self.lbg.append(0.0)
            self.lbg.append(0.0)
            self.lbg.append(-self.omega_max)
            self.ubg.append(0.0)
            self.ubg.append(0.0)
            self.ubg.append(0.0)
            self.ubg.append(self.omega_max)

        # Acceleration Constraint
        for _ in range(self.N-1):
            self.lbg.append(-self.acc_max)
            self.lbg.append(-self.omega_acc_max)
            self.ubg.append(self.acc_max)
            self.ubg.append(self.omega_acc_max)

        # Constraint on X_input [lin vel, ang vel]
        for _ in range(self.N):
            self.lbx.append(0)
            self.lbx.append(-np.pi/2)
            self.ubx.append(self.v_max)
            self.ubx.append(np.pi/2)

        # Constraint on X_state [x, y, theta], unconstrained
        for _ in range(self.N+1):  # note that this is different with the method using structure
            self.lbx.append(-np.inf)
            self.lbx.append(-np.inf)
            self.lbx.append(-np.pi)
            self.ubx.append(np.inf)
            self.ubx.append(np.inf)
            self.ubx.append(np.pi)
        
        # cost function in case is_stop
        nlp_prob_is_stop = {'f': obj_is_stop, 'x': opt_variables, 'p': P, 'g': ca.vertcat(*g)}
        self.solver_is_stop = ca.nlpsol('solver', 'ipopt', nlp_prob_is_stop, opts_setting)
        
        # cost function in case without wheelchair
        obj_no_chair = 0
        g_no_chair = [] # equal constrains
        g_no_chair.append(X[:,0] - P[:3]) # initial state
        for k in range(self.N):
            obj_no_chair = obj_no_chair + ca.mtimes([(X[:,k]-P[3:]).T, Q, X[:,k]-P[3:]]) + ca.mtimes([U[:,k].T, R, U[:,k]])
            g_no_chair.append((X[2, k+1]-X[2, k])/self.T) # angular velocity
        
        # linear and angular acceleration constraints
        for l in range(self.N-1):
            g_no_chair.append((U[0,l+1]-U[0,l])/self.T)   # linear acceleration
            g_no_chair.append(((X[2, l+2]-X[2, l+1])/self.T-(X[2, l+1]-X[2, l])/self.T)/self.T)   # angular acceleration
        
        nlp_prob_no_chair = {'f': obj_no_chair, 'x': opt_variables, 'p': P, 'g': ca.vertcat(*g_no_chair)}
        self.solver_no_chair = ca.nlpsol('solver', 'ipopt', nlp_prob_no_chair, opts_setting)

        # inequality constraints (state constraints)
        self.lbg_no_chair = []   # lower bound of the constraints
        self.ubg_no_chair = []   # upper bound of the constraints

        # Initial state constraint
        self.lbg_no_chair.append(0.0)
        self.lbg_no_chair.append(0.0)
        self.lbg_no_chair.append(0.0)
        self.ubg_no_chair.append(0.0)
        self.ubg_no_chair.append(0.0)
        self.ubg_no_chair.append(0.0)

        # Angular Velocity constraint
        for _ in range(self.N):
            self.lbg_no_chair.append(-self.omega_max)
            self.ubg_no_chair.append(self.omega_max)

        # Acceleration Constraint
        for _ in range(self.N-1):
            self.lbg_no_chair.append(-self.acc_max)
            self.lbg_no_chair.append(-self.omega_acc_max)
            self.ubg_no_chair.append(self.acc_max)
            self.ubg_no_chair.append(self.omega_acc_max)


    def shift_movement(self, T, t0, x0, u, x_f, f):
        f_value = f(x0, u[0, :])
        st = x0 + T*f_value.full()
        t = t0 + T
        u_end = np.concatenate((u[1:], u[-1:]))
        x_f = np.concatenate((x_f[1:], x_f[-1:]), axis=0)

        return t, st, u_end, x_f


    def solve(self, current_time):
        self.is_callback = False

        # initial and goal states
        initial_loc = (0, 0, 0)
        if self.current_theta != 0:
            goal_x, goal_y, goal_theta = abs(self.current_y)/np.sin(abs(self.current_theta)), 0, - self.current_theta
            if abs(goal_theta) < (70 * np.pi / 180) :
                if goal_x > self.v_max*self.T*self.N:
                    goal_x = self.v_max*self.T*self.N
                goal_y = -self.current_y * np.cos(abs(self.current_theta))
        else:
            goal_x, goal_y, goal_theta = 0, 0, 0
        goal_loc = (goal_x, goal_y, goal_theta)

        t0 = 0.0
        x0 = np.array(initial_loc).reshape(-1,1)  # initial state
        x_m = np.zeros((self.n_states, self.N+1))
        next_states = x_m.copy().T
        xs = np.array(goal_loc).reshape(-1,1)   # final state

        # initial control inputs
        if self.first_iter:
            self.u0 = np.array([self.current_vel, self.current_steering]*self.N).reshape(-1,2)
        else:
            self.u0[0] = [self.current_vel, self.current_steering]
        self.first_iter = False

        if self.is_stop:    # immidiately stop as much as possible
            c_p = np.concatenate((x0, xs))
            init_control = np.concatenate((self.u0.reshape(-1, 1), next_states.reshape(-1, 1)))
            res = self.solver_is_stop(x0=init_control, p=c_p, lbg=self.lbg, lbx=self.lbx, ubg=self.ubg, ubx=self.ubx)
            estimated_opt = res['x'].full()
            self.u0 = estimated_opt[:self.n_controls*self.N].reshape(self.N, self.n_controls)  # (N, n_controls)
            x_m = estimated_opt[self.n_controls*self.N:].reshape(self.N+1, self.n_states)  # (N+1, n_states)

        elif self.no_chair: # maneuver without wheelchair
            c_p = np.concatenate((x0, xs))
            init_control = np.concatenate((self.u0.reshape(-1, 1), next_states.reshape(-1, 1)))
            res = self.solver_no_chair(x0=init_control, p=c_p, lbg=self.lbg_no_chair, lbx=self.lbx, ubg=self.ubg_no_chair, ubx=self.ubx)
            estimated_opt = res['x'].full()
            self.u0 = estimated_opt[:self.n_controls*self.N].reshape(self.N, self.n_controls)  # (N, n_controls)
            x_m = estimated_opt[self.n_controls*self.N:].reshape(self.N+1, self.n_states)  # (N+1, n_states)

        else:   # maneuver with wheelchair
            # Move straight when the robot close enough to the goal line
            if np.linalg.norm(x0-xs) <= 1e-2:
                self.u0 = np.array([[self.v_max, 0]]*self.N)
                x_m = np.hstack(((self.control_time * self.v_max).reshape(-1, 1), np.zeros((41,2))))
                # print('mpc linear moving')

            # start MPC
            else :
                # one_iter_time = time.time()
                # set parameter
                c_p = np.concatenate((x0, xs))
                init_control = np.concatenate((self.u0.reshape(-1, 1), next_states.reshape(-1, 1)))
                # t_ = time.time()
                res = self.solver(x0=init_control, p=c_p, lbg=self.lbg, lbx=self.lbx, ubg=self.ubg, ubx=self.ubx)
                # the feedback is in the series [u0, x0, u1, x1, ...]
                estimated_opt = res['x'].full()
                self.u0 = estimated_opt[:self.n_controls*self.N].reshape(self.N, self.n_controls)  # (N, n_controls)
                x_m = estimated_opt[self.n_controls*self.N:].reshape(self.N+1, self.n_states)  # (N+1, n_states)
                # print(f"one iteration time: {time.time() - one_iter_time}")
        
        time_array = (self.control_time + current_time).reshape(-1,1)
        vel_command, steering_angle = self.u0[:,0].reshape(-1,1), self.u0[:,1].reshape(-1,1)
        self.control_output = np.hstack((time_array[:self.N], np.hstack((np.hstack((vel_command * np.cos(steering_angle), vel_command * np.sin(steering_angle))), 
            vel_command * np.sin(steering_angle) / self.l_wh))))
        self.perception_output = np.hstack((time_array, x_m))

        return self.perception_output, self.control_output




