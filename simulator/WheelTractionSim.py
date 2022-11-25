"""
This is simulator for 보산진 과제
"""
import math
import os
import sys
from copy import copy
import yaml
from pygame import Rect

sys.path.append('../')
from simulator.utils import *
import casadi as ca


class WheelTractionSim:
    def __init__(self,
                 settings_yaml,
                 x_init=0, y_init=0,  # meter
                 yaw_init=0,  # rad
                 ):
        assert os.path.isfile(settings_yaml), settings_yaml
        with open(settings_yaml, 'r') as stream:
            settings = yaml.safe_load(stream)
            self.L = settings['L']
            self.a = settings['a']
            self.b = settings['b']
            self.wheel_width = settings['wheel_width']
            self.chair_wheel = settings['chair_wheel_width']
            self.wheel_dia = settings['wheel_dia']
            self.wheeltowheel = settings['wheeltowheel']
            self.chair_a = settings['chair_a']
            self.chair_b = settings['chair_b']
            self.chair_wheel_dia = settings['chair_wheel_dia']
            self.max_acc = settings['max_acc']
            self.max_vel = settings['max_vel']
            self.max_ang_acc = settings['max_ang_acc'] * pi / 180  # to radian
            self.max_ang_vel = settings['max_ang_vel'] * pi / 180  # to radian
            self.rend_size = settings['rend_size']
            self.dt = settings['dt']
            self.focusing_speed = settings['focusing_speed']
            self.marker_size = settings['marker_size']
            self.with_chair = settings['start_with_chair']
            self.render = settings['render']
            self.env_size = settings['env_size']
            self.traj_len = settings['traj_len']

        # define robot param
        self.x_vel = 0  # meter/sec
        self.y_vel = 0  # meter/sec
        self.yaw_rate = 0  # rad/sec
        self.trajectory_queue = np.empty((0, 2))
        self.oT_r = Trans(Rot(yaw_init), Vector2(x_init, y_init))
        self.robot_vel_prev = self.robot_vel = Trans(Rot(0), Vector2(0, 0))
        self.robot_acc = Trans(Rot(0), Vector2(0, 0))
        self.pred_traj = None
        self._pred_traj_old = None
        self.prev_exe_time = None

        # define robot dynamics
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        theta = ca.SX.sym('theta')
        states = ca.vertcat(x, y, theta)
        vy = ca.SX.sym('vy')
        vx = ca.SX.sym('vx')
        omega = ca.SX.sym('omega')
        controls = ca.vertcat(vx, vy, omega)
        rhs = ca.vertcat(vx * ca.cos(theta) - vy * ca.sin(theta), vx * ca.sin(theta) + vy * ca.cos(theta), omega)
        self.dynamics = ca.Function('dynamics', [states, controls], [rhs], ['state', 'controls'], ['rhs'])

        # define wheelchair param
        self.wT_r = Trans(Rot(0), Vector2(self.L, 0))
        self.rT_w = Trans(Rot(0), Vector2(-self.L, 0))
        self.docking(self.with_chair)

        self.sim_time = 0

        # define line coordinate
        # l : guide line, nextnode : end of guide line
        self.oT_l = Trans(Rot(0), Vector2(0, 0))
        lT_nextnode = Trans(Rot(0), Vector2(3, 0))
        self.lT_r = trans_from_mat(np.linalg.inv(self.oT_l.as_mat()) @ self.oT_r.as_mat())
        self.oT_nextnode = trans_from_mat(self.oT_l.as_mat() @ lT_nextnode.as_mat())

        self.render_center = Vector2(0, 0)

        if self.render:
            pygame.init()
            pygame.display.set_caption("Episode Render")
            self.py_map = pygame.display.set_mode((self.rend_size, self.rend_size))
            self.rend_ratio = self.rend_size / self.env_size
            self.rend_wc()

        print("hello wheel chair")

    def step(self, x_vel=None, y_vel=None, yaw_rate=None, pred_traj=None, exec_time=None):
        """
        update vehicle position(translation matrix) with given ackerman params
        :param x_vel y_vel yaw_rate: input params without wheelchair?
        :param yaw_rate: command rotation velocity of ackerman dynamics
        :param exec_time: execution time in sec float
        :return: y, theta
        """
        if pred_traj is not None:
            self.pred_traj = pred_traj
        update_command_vel = True
        if yaw_rate is None or y_vel is None or yaw_rate is None:
            update_command_vel = False
        elif sqrt(x_vel ** 2 + y_vel ** 2) > self.max_vel:
            print(
                f"[WARNING] Command velocity exceeded maximum speed. Violation : {sqrt(x_vel ** 2 + y_vel ** 2) - self.max_vel} m/s")
            update_command_vel = False
        elif abs(yaw_rate) > self.max_ang_vel:
            print(
                f"[WARNING] Command yaw_rate exceeded maximum raw_rate. Violation : {abs(yaw_rate) - self.max_ang_vel}")
            update_command_vel = False

        if update_command_vel:
            # Check acceleration constraint
            if self.prev_exe_time is None:
                self.prev_exe_time = exec_time - self.dt
            yaw_acc = abs(yaw_rate - self.robot_vel_prev.rotation.yaw) / (exec_time - self.prev_exe_time)
            lin_acc = (np.array([[x_vel], [y_vel]]) - self.robot_vel_prev.position.as_vec()) / (
                    exec_time - self.prev_exe_time)
            self.prev_exe_time = exec_time
            lin_acc_norm = np.linalg.norm(lin_acc)
            if yaw_acc > self.max_ang_acc:
                print(
                    f"[WARNING] Command yaw_rate exceeded maximum angular acceleration. Violation : {yaw_acc - self.max_ang_acc:.4f}")
                # adjust yaw_rate with maximum angular acceleration
                # yaw_rate = self.robot_vel_prev.rotation.yaw + self.dt * self.max_ang_acc
            if lin_acc_norm > self.max_acc:
                print(
                    f"[WARNING] Command velocity exceeded maximum acceleration. Violation : {lin_acc_norm},  {self.max_acc:.4f}")
                # print(f" acc : {lin_acc_norm}, {lin_acc_test} ")
                # lin_vel = self.robot_vel_prev.position.as_vec() + \
                #           self.dt * self.max_acc * lin_acc / lin_acc_norm
                # x_vel, y_vel = lin_vel.squeeze()
                # print(lin_acc_norm)

        if self.with_chair:
            """
            from ackerman dynamics,
            x_dot_r = R_ack * Yaw_Rate
            y_dot_r = L * Yaw_Rate
            theta_dot_r = Yaw_Rate 
            """
            if abs(y_vel - self.L * yaw_rate) > 1e-3:
                # it's slip condition ! command should satisfy ackerman constraint
                print(
                    f"[WARNING] Yaw Rate and Y velocity does not satisfy Ackerman Constraint. Violation : {abs(y_vel - self.L * yaw_rate)}")
                # ignore the input
                # update_command_vel = False

            if update_command_vel:
                self.yaw_rate = yaw_rate
                self.x_vel = x_vel
                self.y_vel = y_vel
                phi = atan2(self.y_vel, self.x_vel)
                self.r_ack = self.L * math.tan(pi / 2 - phi)

        else:  # Undocking mode
            if update_command_vel:
                self.yaw_rate = yaw_rate
                self.x_vel = x_vel
                self.y_vel = y_vel
        if self.x_vel < 0:
            # If sign of rotation velocity and radius of and ackerman dynamics means
            print("[WARNING] Moving backward!!!")
        self._update()

    def docking(self, is_dock):
        self.with_chair = is_dock
        if self.with_chair:
            self.oT_w = trans_from_mat(self.oT_r.as_mat() @ self.rT_w.as_mat())
            self.r_ack = 0  # meter

        else:
            self.oT_w = None
            self.r_ack = None  # meter
        self.x_vel = 0
        self.y_vel = 0
        self.yaw_rate = 0

    def _update(self):
        """
        Updates all variables with dt, state_dot by following equation
        -> X_new = X_old + dt * X_dot
        Returns:
            None
        """
        # Update translation of robot/wheelchair
        # rT_newr = Trans(Rot(self.yaw_rate * self.dt), Vector2(self.x_vel * self.dt, self.y_vel * self.dt))
        # Use Runge-Kutta method
        input_u = [self.x_vel, self.y_vel, self.yaw_rate]
        rT_r = [0, 0, 0]
        k1 = self.dynamics(rT_r, [self.x_vel, self.y_vel, self.yaw_rate])
        k2 = self.dynamics(rT_r + self.dt / 2 * k1, input_u)
        k3 = self.dynamics(rT_r + self.dt / 2 * k2, input_u)
        k4 = self.dynamics(rT_r + self.dt * k3, input_u)
        rT_newr = rT_r + self.dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        rT_newr = rT_newr.toarray().squeeze()
        rT_newr = Trans(Rot(rT_newr[2]), Vector2(rT_newr[0], rT_newr[1]))
        self.oT_r = trans_from_mat(self.oT_r.as_mat() @ rT_newr.as_mat())
        if self.with_chair:
            self.oT_w = trans_from_mat(self.oT_r.as_mat() @ self.rT_w.as_mat())

        self.lT_r = trans_from_mat(np.linalg.inv(self.oT_l.as_mat()) @ self.oT_r.as_mat())
        # print(f"error_y : {self.lT_r.position.y_val}, error_theta : {self.lT_r.rotation.yaw}")

        # Update velocity/acceleration of robot
        self.robot_vel_prev = copy(self.robot_vel)
        self.robot_vel = Trans(Rot(self.yaw_rate), Vector2(self.x_vel, self.y_vel))
        yaw_acc = (self.robot_vel.rotation.yaw - self.robot_vel_prev.rotation.yaw) / self.dt
        lin_acc = (self.robot_vel.position.as_vec() - self.robot_vel_prev.position.as_vec()) / self.dt
        self.robot_acc = Trans(Rot(yaw_acc), Vector2(lin_acc[0], lin_acc[1]))

        self.trajectory_queue = np.append(self.trajectory_queue,
                                          np.array([[self.oT_r.position.x_val, self.oT_r.position.y_val]]), axis=0)
        if len(self.trajectory_queue) > self.traj_len:
            self.trajectory_queue = self.trajectory_queue[1:, :]

        # focusing update
        self.render_center.x_val = self.render_center.x_val * (1 - self.focusing_speed) + \
                                   self.oT_r.position.x_val * self.focusing_speed
        self.render_center.y_val = self.render_center.y_val * (1 - self.focusing_speed) + \
                                   self.oT_r.position.y_val * self.focusing_speed
        self.sim_time += self.dt
        if self.render:
            self.rend_wc()

    def _visualize_with_rostopic(self, l_y_r, l_theta_r):
        self.oT_r = Trans(Rot(0), Vector2(0, 0))
        self.lT_r = Trans(Rot(l_theta_r), Vector2(0, l_y_r))
        self.oT_l = trans_from_mat(self.oT_r.as_mat() @ np.linalg.inv(self.lT_r.as_mat()))
        lT_nextnode = Trans(Rot(0), Vector2(10, 0))
        self.oT_nextnode = trans_from_mat(self.oT_l.as_mat() @ lT_nextnode.as_mat())
        if self.with_chair:
            self.oT_w = trans_from_mat(self.oT_r.as_mat() @ self.rT_w.as_mat())

        # self.lT_r = trans_from_mat(np.linalg.inv(self.oT_l.as_mat()) @ self.oT_r.as_mat())

        # focusing update
        self.render_center.x_val = 0
        self.render_center.y_val = 0
        if self.render:
            self.rend_wc()

    def rend_wc(self):
        """
        Render the wheelchair simulation.
        Returns:
        """
        self.py_map.fill((255, 255, 255))
        # Draw projection region
        rTproj_corn_list = [Trans(Rot(0), Vector2(1.2069, 1.0221)),
                            Trans(Rot(0), Vector2(0.2268, 0.4529)),
                            Trans(Rot(0), Vector2(0.2524, -0.4908)),
                            Trans(Rot(0), Vector2(1.3329, -1.0717))]
        points = []
        for rTproj_corn in rTproj_corn_list:
            point = trans_from_mat(self.oT_r.as_mat() @ rTproj_corn.as_mat())
            points.append(point.position.as_vec().squeeze().tolist())
        points = self._sur_coord(points)
        pygame.draw.polygon(self.py_map, (255, 255, 0), points)

        # Draw line & marker
        pygame.draw.line(self.py_map, (0, 0, 0), self._sur_coord((self.oT_l.position.x_val, self.oT_l.position.y_val)),
                         self._sur_coord((self.oT_nextnode.position.x_val, self.oT_nextnode.position.y_val)))
        points = rectangle_points(self.oT_nextnode.position.x_val, self.oT_nextnode.position.y_val, dy=self.marker_size,
                                  dx=self.marker_size, rotation=self.oT_nextnode.rotation.yaw)
        points = self._sur_coord(points)
        pygame.draw.polygon(self.py_map, (255, 0, 255), points)

        # render robot, rw : robot wheel
        points = rectangle_points(self.oT_r.position.x_val, self.oT_r.position.y_val, dy=self.b * 2,
                                  dx=self.a * 2, rotation=self.oT_r.rotation.yaw)
        points = self._sur_coord(points)
        pygame.draw.polygon(self.py_map, (125, 0, 0), points)

        rT_rw_list = [Trans(Rot(0), Vector2(self.a, self.b)),
                      Trans(Rot(0), Vector2(self.a, -self.b)),
                      Trans(Rot(0), Vector2(- self.a, self.b)),
                      Trans(Rot(0), Vector2(- self.a, -self.b))]

        for rT_rw in rT_rw_list:
            oT_rw = trans_from_mat(self.oT_r.as_mat() @ rT_rw.as_mat())
            points = rectangle_points(oT_rw.position.x_val, oT_rw.position.y_val, dy=self.wheel_width,
                                      dx=self.wheel_dia, rotation=oT_rw.rotation.yaw)
            points = self._sur_coord(points)
            pygame.draw.polygon(self.py_map, (0, 0, 0), points)

        # render wheelchair, ww : wheelchair wheel
        if self.with_chair:
            points = rectangle_points(self.oT_w.position.x_val, self.oT_w.position.y_val, dy=self.chair_b,
                                      dx=self.chair_a, rotation=self.oT_w.rotation.yaw)

            points = self._sur_coord(points)
            pygame.draw.polygon(self.py_map, (0, 255, 0), points)

            # render wheelchair, ww : wheelchair wheel
            wT_ww_list = [Trans(Rot(0), Vector2(0, self.wheeltowheel / 2)),
                          Trans(Rot(0), Vector2(0, -self.wheeltowheel / 2))]
            for wT_ww in wT_ww_list:
                oT_ww = trans_from_mat(self.oT_w.as_mat() @ wT_ww.as_mat())
                points = rectangle_points(oT_ww.position.x_val, oT_ww.position.y_val, dy=self.wheel_width,
                                          dx=self.chair_wheel_dia, rotation=oT_ww.rotation.yaw)
                points = self._sur_coord(points)
                pygame.draw.polygon(self.py_map, (0, 0, 0), points)

            # Draw ackerman motion
            # pinpoints' position 'p'
            wT_p = Trans(Rot(0), Vector2(0, self.r_ack))
            oT_p = trans_from_mat(self.oT_w.as_mat() @ wT_p.as_mat())
            pygame.draw.line(self.py_map, (0, 100, 255),
                             self._sur_coord((oT_p.position.x_val, oT_p.position.y_val)),
                             self._sur_coord((self.oT_w.position.x_val, self.oT_w.position.y_val)))
            # arc of motion
            rect_start = (oT_p.position.x_val - abs(self.r_ack), oT_p.position.y_val + abs(self.r_ack))
            rect = Rect(self._sur_coord(rect_start),
                        (abs(2 * self.r_ack * self.rend_ratio), abs(2 * self.r_ack * self.rend_ratio)))
            start_angle = -math.copysign(pi * 0.5, self.r_ack) + self.oT_w.rotation.yaw
            end_angle = start_angle + self.yaw_rate
            # if self.yaw_rate < 0:
            #     pygame.draw.arc(self.py_map, (0, 0, 0), rect, end_angle, start_angle)
            # else:
            #     pygame.draw.arc(self.py_map, (0, 0, 0), rect, start_angle, end_angle)

        # Draw trajectory history
        if len(self.trajectory_queue) != 0:
            traj = self.trajectory_queue[0:-1:int(self.traj_len / 10)].copy()
            for point in traj:
                pygame.draw.circle(self.py_map, (50, 50, 50), self._sur_coord(point), 1)

        # Draw prediction trajectory
        if self.pred_traj is not None:
            temp = np.transpose(
                self.oT_r.as_mat() @ np.transpose(
                    np.hstack((self.pred_traj[:, 1:3], np.ones((len(self.pred_traj), 1))))))
            start_xy = []
            end_xy = []
            end_xy2 = []
            for pose, theta in zip(temp, self.pred_traj[:, 3]):
                start_xy.append((pose[0], pose[1]))
                end_xy.append((pose[0] + 0.5 * cos(theta + self.oT_r.rotation.yaw),
                               pose[1] + 0.5 * sin(theta + self.oT_r.rotation.yaw)))
                end_xy2.append((pose[0] + 0.5 * sin(theta + self.oT_r.rotation.yaw),
                                pose[1] - 0.5 * cos(theta + self.oT_r.rotation.yaw)))
            self._pred_traj_old = {'start_xy': start_xy.copy(), 'end_xy': end_xy.copy(), 'end_xy2': end_xy2.copy()}
            # self.pred_traj = None
        if self._pred_traj_old is not None:
            for start, end, end2 in zip(self._pred_traj_old['start_xy'], self._pred_traj_old['end_xy'],
                                        self._pred_traj_old['end_xy2']):
                pygame.draw.line(self.py_map, (255, 0, 0), self._sur_coord(start), self._sur_coord(end))
                pygame.draw.line(self.py_map, (0, 0, 255), self._sur_coord(start), self._sur_coord(end2))
        pygame.display.update()

    def _sur_coord(self, point):
        """
        transform to pygame surface coordinate
        :param point, point in world coordinate
        :return:
        """
        if type(point) is tuple or type(point) is list:
            if type(point[0]) is list:
                res = []
                for p in point:
                    x, y = p
                    x = (x - self.render_center.x_val) * self.rend_ratio + self.rend_size / 2
                    y = -(y - self.render_center.y_val) * self.rend_ratio + self.rend_size / 2
                    res.append([x, y])
                return res
            x, y = point
            x = (x - self.render_center.x_val) * self.rend_ratio + self.rend_size / 2
            y = -(y - self.render_center.y_val) * self.rend_ratio + self.rend_size / 2
            return (x, y)
        if type(point) is np.ndarray:
            if point.ndim == 2:
                point[:, 0] = ((point[:, 0] - self.render_center.x_val) * self.rend_ratio + self.rend_size / 2).copy()
                point[:, 1] = (-(point[:, 1] - self.render_center.y_val) * self.rend_ratio + self.rend_size / 2).copy()
            elif point.ndim == 1:
                point[0] = ((point[0] - self.render_center.x_val) * self.rend_ratio + self.rend_size / 2).copy()
                point[1] = (-(point[1] - self.render_center.y_val) * self.rend_ratio + self.rend_size / 2).copy()
            return point

    def reset_line(self, theta=None, guideline_length=3):
        """
        Update guideline with given angle
        Args:
            theta: angle of new guideline
            guideline_length: length of new guideline

        Returns:
            None
        """
        assert guideline_length > 0, "Guide line length must be positive."
        if theta is None:
            theta = (self.oT_l.rotation.yaw + pi / 2) % (2 * pi)
        else:
            theta = (theta) % (2 * pi)
        self.oT_l = Trans(Rot(theta), self.oT_nextnode.position)
        lT_nextnode = Trans(Rot(0), Vector2(guideline_length, 0))
        self.oT_nextnode = trans_from_mat(self.oT_l.as_mat() @ lT_nextnode.as_mat())
