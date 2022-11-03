import casadi as ca
import numpy as np
import time
import matplotlib.pyplot as plt

class ackermann_mpc():
    def __init__(self):
        self.T = 0.05   # sampling time [s]
        self.N = 40     # prediction horizon
        self.l_wh = 0.7  # [m]
        self.v_max = 0.5 # [m/s]
        self.acc_max = 0.3 # [m/s^2]
        self.omega_max = 15 * np.pi / 180    # [rad/s]
        self.omega_acc_max = 10 * np.pi / 180    # [rad/s^2]
        
        self.first_iter = True
        self.start_time = time.time()
        self.states_for_visualize = []
        self.index_t = []
        self.is_stop = False

        self.create_mpc_model()
        # self.ackermann_mpc()

    def state_update(self, current_y, current_theta, current_vel, current_omega):
        self.current_y = current_y
        self.current_theta = current_theta
        self.current_vel = current_vel
        if self.current_vel == 0:
            self.current_steering = 0
        else:
            self.current_steering = np.arcsin(current_omega*self.l_wh/self.current_vel)
        self.is_callback = True

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
        self.f = ca.Function('f', [states, controls], [rhs], ['input_state', 'control_input'], ['rhs'])

        # for MPC
        U = ca.SX.sym('U', self.n_controls, self.N)
        X = ca.SX.sym('X', self.n_states, self.N+1)   # state predictions are stored in this matrix
        P = ca.SX.sym('P', self.n_states + self.n_states)  # parameters which include the initial (in every step) and the reference state of the robot

        # define
        Q = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 3.0]])
        R = np.array([[1.0, 0.0], [0.0, 0.0]])

        # cost function
        obj = 0 # cost
        g = []  # equal constrains
        g.append(X[:,0] - P[:3]) # initial state
        for i in range(self.N):
            obj = obj + ca.mtimes([(X[:,i]-P[3:]).T, Q, X[:,i]-P[3:]]) + ca.mtimes([U[:,i].T, R, U[:,i]])
            x_next_ = self.f(X[:,i], U[:,i])*self.T + X[:,i]
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
        self.lbg = []   # lower bount of the constraints
        self.ubg = []   # upper bount of the constraints
        self.lbx = []   # decision variables lower bound
        self.ubx = []   # decision variables upper bound

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
        for _ in range(self.N-1):
            self.lbg.append(-self.acc_max)
            self.lbg.append(-self.omega_acc_max)
            self.ubg.append(self.acc_max)
            self.ubg.append(self.omega_acc_max)
        for _ in range(self.N):
            self.lbx.append(0)
            self.lbx.append(-np.pi/2)
            self.ubx.append(self.v_max)
            self.ubx.append(np.pi/2)
        for _ in range(self.N+1):  # note that this is different with the method using structure
            self.lbx.append(-np.inf)
            self.lbx.append(-np.inf)
            self.lbx.append(-np.pi)
            self.ubx.append(np.inf)
            self.ubx.append(np.inf)
            self.ubx.append(np.pi)

    def shift_movement(self, T, t0, x0, u, x_f, f):
        f_value = f(x0, u[0, :])
        st = x0 + T*f_value.full()
        t = t0 + T
        # print(u[:,0])
        # u_end = np.concatenate((u[:, 1:], u[:, -1:]), axis=1)
        u_end = np.concatenate((u[1:], u[-1:]))
        # x_f = np.concatenate((x_f[:, 1:], x_f[:, -1:]), axis=1)
        x_f = np.concatenate((x_f[1:], x_f[-1:]), axis=0)

        return t, st, u_end, x_f

    def ackermann_mpc(self):
        self.is_callback = False
        initial_loc = (0, 0, 0)
        if self.current_theta != 0:
            # goal_x, goal_y, goal_theta = self.current_y/np.sin(abs(self.current_theta)), 0, - self.current_theta
            goal_x, goal_y, goal_theta = self.current_y/np.sin(abs(self.current_theta)), 0, - self.current_theta
            if abs(goal_theta) < (45 * np.pi / 180) :
                goal_x = self.v_max*self.T*self.N
                goal_y = -self.current_y
        elif self.current_y == 0:
            goal_x, goal_y, goal_theta = 0, 0, 0
        else:
            goal_x, goal_y, goal_theta = self.v_max*self.T*self.N, -self.current_y, -self.current_y
        goal_loc = (goal_x, goal_y, goal_theta)

        t0 = 0.0
        x0 = np.array(initial_loc).reshape(-1,1)  # initial state
        x0_ = x0.copy()
        x_m = np.zeros((self.n_states, self.N+1))
        next_states = x_m.copy().T
        xs = np.array(goal_loc).reshape(-1,1)   # final state
        if self.first_iter:
            self.u0 = np.array([self.current_vel, self.current_steering]*self.N).reshape(-1,2)
        else:
            self.u0[0] = [self.current_vel, self.current_steering]
        self.first_iter = False
        # else:
        #     self.u0 = self.u0
        self.x_c = []  # contains for the history of the state
        self.u_c = []
        self.t_c = []  # for the time
        sim_time = 20.0
        mpciter = 0

        # start MPC
        while(np.linalg.norm(x0-xs) > 1e-2): # and self.mpciter-sim_time/self.T < 0.0):
            one_iter_time = time.time()
            # set parameter
            c_p = np.concatenate((x0, xs)) #, human_trajectory[:,mpciter:mpciter+N+1]))
            # print(u0.reshape(-1, 1))
            init_control = np.concatenate((self.u0.reshape(-1, 1), next_states.reshape(-1, 1)))
            t_ = time.time()
            res = self.solver(x0=init_control, p=c_p, lbg=self.lbg, lbx=self.lbx, ubg=self.ubg, ubx=self.ubx)
            self.index_t.append(time.time() - t_)
            # the feedback is in the series [u0, x0, u1, x1, ...]
            estimated_opt = res['x'].full()
            self.u0 = estimated_opt[:self.n_controls*self.N].reshape(self.N, self.n_controls)  # (N, n_controls)
            x_m = estimated_opt[self.n_controls*self.N:].reshape(self.N+1, self.n_states)  # (N+1, n_states)
            self.x_c.append(x_m.T)
            self.u_c.append(self.u0[0, :])
            self.t_c.append(t0)
            t0, x0, self.u0, next_states = self.shift_movement(self.T, t0, x0, self.u0, x_m, self.f)
            x0 = ca.reshape(x0, -1, 1)
            x0 = x0.full()
            self.states_for_visualize.append(x0)
            self.current_y = goal_x - x0[0]
            self.current_theta = x0[2] - goal_theta
            xs = np.array([x0[0] + self.current_y/np.sin(abs(self.current_theta)), x0[1], np.array([goal_theta])]).reshape(-1,1)
            # print(u0[0])
            mpciter += 1
            print(f"one iteration time: {time.time() - one_iter_time}")

            # for 승우 simulation
            self.for_simulation_command = self.u_c[mpciter-1]
            return 0

            if self.is_callback == True:
                mpciter = 0
                initial_loc = (0, 0, 0)
                goal_x, goal_y, goal_theta = self.current_y/np.sin(abs(self.current_theta)), 0, - self.current_theta
                goal_loc = (goal_x, goal_y, goal_theta)

                t0 = 0.0
                x0 = np.array(initial_loc).reshape(-1,1)  # initial state
                x0_ = x0.copy()
                x_m = np.zeros((self.n_states, self.N+1))
                next_states = x_m.copy().T
                xs = np.array(goal_loc).reshape(-1,1)   # final state

                self.u0[0] = [self.current_vel, self.current_steering]
                self.x_c = []  # contains for the history of the state
                self.u_c = []
                self.t_c = []  # for the time
                self.is_callback = False

        while np.linalg.norm(x0-xs) <= 1e-2:
            self.for_simulation_command = np.array([self.v_max, 0])
            print('mpc linear moving')
            return 0
        # t_v = np.array(self.index_t)
        # self.states_for_visualize = np.squeeze(np.array(self.states_for_visualize), axis = 2)

        # print("solver average time:", t_v.mean())
        # print("whole mpc iteration average time:", (time.time() - self.start_time)/(self.mpciter))
        # print("initial robot location:", initial_loc)
        # print("final robot location:", self.states_for_visualize[-1,:])
        # print("final goal x:", goal_x, ", final goal theta:", goal_theta)
        # print("final goal error(meter):", goal_x - self.states_for_visualize[-1,0], ", final theta error(degree):", (goal_theta - self.states_for_visualize[-1,2])*180/np.pi)
        # print('end')

if __name__ == "__main__":
    # Simulation
    img_size_x, img_size_y = 10, 10
    img_size = (img_size_x, img_size_y)
    robot_x, robot_y, robot_theta = img_size_x / 2, img_size_y / 2, 0
    # robot_loc = (robot_x, robot_y, robot_theta)
    ys = 1.5 # meter
    theta_s = 90 * np.pi / 180  # rad

    my_model = ackermann_mpc(init_y=ys, init_theta=theta_s, init_vel=0.5)

    plt.plot(my_model.states_for_visualize[:,0]+robot_x, my_model.states_for_visualize[:,1]+robot_y)
    plt.xlim(0, img_size[0])
    plt.ylim(0, img_size[1])
    plt.show()

    print('end')