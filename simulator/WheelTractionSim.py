"""
This is simulator for 보산진 과제
"""
import math
import os
from copy import copy
import yaml
import time
from pygame import Rect
from utils import *
import sys
sys.path.append('/home/syeon/bosanjin')
import mpc_ackermann_steering_func as mpc


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

        # define robot param
        self.x_vel = 0  # meter/sec
        self.y_vel = 0  # meter/sec
        self.yaw_rate = 0  # rad/sec
        self.trajectory_queue = np.empty((0, 2))
        self.oT_r = Trans(Rot(yaw_init), Vector2(x_init, y_init))
        self.robot_vel_prev = self.robot_vel = Trans(Rot(0), Vector2(0, 0))
        self.robot_acc = Trans(Rot(0), Vector2(0, 0))

        # define wheelchair param
        self.wT_r = Trans(Rot(0), Vector2(self.L, 0))
        self.rT_w = Trans(Rot(0), Vector2(-self.L, 0))
        self.docking(self.with_chair)

        self.sim_time = 0

        # define line coordinate
        # l : guide line, nextnode : end of guide line
        self.oT_l = Trans(Rot(0), Vector2(0, 0))
        lT_nextnode = Trans(Rot(0), Vector2(10, 0))
        self.lT_r = trans_from_mat(np.linalg.inv(self.oT_l.as_mat()) @ self.oT_r.as_mat())
        self.oT_nextnode = trans_from_mat(self.oT_l.as_mat() @ lT_nextnode.as_mat())

        self.render_center = Vector2(0, 0)

        if self.render:
            pygame.init()
            pygame.display.set_caption("Episode Render")
            self.py_map = pygame.display.set_mode((self.rend_size, self.rend_size))
            self.rend_ratio = self.rend_size / self.env_size
            self.rend_wc()

    def step(self, x_vel=None, y_vel=None, yaw_rate=None):
        """
        update vehicle position(translation matrix) with given ackerman params
        :param x_vel y_vel yaw_rate: input params without wheelchair?
        :param yaw_rate: command rotation velocity of ackerman dynamics
        :return: y, theta
        """
        # TODO Done : w/ & w/o case 모두에 대해서 input 통일하고 w/ case에서 ackerman constraint 만족하는지만 체크하고 input 넣는걸로!
        update_command_vel = True
        if yaw_rate is None or y_vel is None or yaw_rate is None:
            update_command_vel = False
        elif sqrt(x_vel ** 2 + y_vel ** 2) > self.max_vel:
            print("[WARNING] Command velocity exceeded maximum speed.")
            update_command_vel = False
        elif abs(yaw_rate) > self.max_ang_vel:
            print("[WARNING] Command yaw_rate exceeded maximum speed.")
            update_command_vel = False

        # TODO : Check acceleration constarint
        # if update_command_vel:
        #     # Check acceleration constraint
        #     yaw_acc = (yaw_rate - self.robot_vel_prev.rotation.yaw) / self.dt
        #     lin_acc = (np.array([[x_vel], [y_vel]]) - self.robot_vel_prev.position.as_vec()) / self.dt
        #     lin_acc_norm = np.linalg.norm(lin_acc)
        #     if yaw_acc > self.max_ang_acc:
        #         # adjust yaw_rate with maximum angular acceleration
        #         # print("[WARNING] Command yaw_rate exceeded maximum angular acceleration.")
        #         yaw_rate = self.robot_vel_prev.rotation.yaw + self.dt * self.max_ang_acc
        #     if lin_acc_norm > self.max_acc:
        #         # print("[WARNING] Command velocity exceeded maximum acceleration.")
        #         lin_vel = self.robot_vel_prev.position.as_vec() + \
        #                   self.dt * self.max_acc * lin_acc / lin_acc_norm
        #         x_vel, y_vel = lin_vel.squeeze()
        #         # print(lin_acc_norm)

        if self.with_chair:
            """
            from ackerman dynamics,
            x_dot_r = R_ack * Yaw_Rate
            y_dot_r = L * Yaw_Rate
            theta_dot_r = Yaw_Rate 
            """
            if abs(y_vel - self.L * yaw_rate) > 1e-4:
                # it's slip condition ! command should satisfy ackerman constraint
                print("[WARNING] Yaw Rate and Y velocity does not satisfy Ackerman Constraint")
                # ignore the input
                update_command_vel = False

            if update_command_vel:
                self.yaw_rate = yaw_rate
                self.x_vel = x_vel
                self.y_vel = y_vel
                phi = atan2(self.y_vel, self.x_vel)
                self.r_ack = self.L * math.tan(pi / 2 - phi)

            if self.x_vel < 0:
                # If sign of rotation velocity and radius of and ackerman dynamics means
                print("[WARNING] Moving backward!!!")
        else:  # Undocking mode
            if update_command_vel:
                self.yaw_rate = yaw_rate
                self.x_vel = x_vel
                self.y_vel = y_vel

        self._update()
        self.lT_r = trans_from_mat(np.linalg.inv(self.oT_l.as_mat()) @ self.oT_r.as_mat())

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
        # Update translation of robot/wheelchair
        rT_newr = Trans(Rot(self.yaw_rate * self.dt), Vector2(self.x_vel * self.dt, self.y_vel * self.dt))
        self.oT_r = trans_from_mat(self.oT_r.as_mat() @ rT_newr.as_mat())
        if self.with_chair:
            self.oT_w = trans_from_mat(self.oT_r.as_mat() @ self.rT_w.as_mat())

        # Update velocity/acceleration of robot
        self.robot_vel_prev = copy(self.robot_vel)
        self.robot_vel = Trans(Rot(self.yaw_rate), Vector2(self.x_vel, self.y_vel))
        yaw_acc = (self.robot_vel.rotation.yaw - self.robot_vel_prev.rotation.yaw) / self.dt
        lin_acc = (self.robot_vel.position.as_vec() - self.robot_vel_prev.position.as_vec()) / self.dt
        self.robot_acc = Trans(Rot(yaw_acc), Vector2(lin_acc[0], lin_acc[1]))

        self.trajectory_queue = np.append(self.trajectory_queue,
                                          np.array([[self.oT_r.position.x_val, self.oT_r.position.y_val]]), axis=0)
        if len(self.trajectory_queue) > 100:
            self.trajectory_queue = self.trajectory_queue[1:, :]

        # focusing update
        self.render_center.x_val = self.render_center.x_val * (1 - self.focusing_speed) + \
                                   self.oT_r.position.x_val * self.focusing_speed
        self.render_center.y_val = self.render_center.y_val * (1 - self.focusing_speed) + \
                                   self.oT_r.position.y_val * self.focusing_speed
        self.sim_time += self.dt
        if self.render:
            self.rend_wc()

    def rend_wc(self):
        self.py_map.fill((255, 255, 255))
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
        pygame.draw.polygon(self.py_map, (255, 0, 0), points)

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
            test = pygame.draw.line(self.py_map, (0, 100, 255),
                                    self._sur_coord((oT_p.position.x_val, oT_p.position.y_val)),
                                    self._sur_coord((self.oT_w.position.x_val, self.oT_w.position.y_val)))
            # arc of motion
            rect_start = (oT_p.position.x_val - abs(self.r_ack), oT_p.position.y_val + abs(self.r_ack))
            rect = Rect(self._sur_coord(rect_start),
                        (abs(2 * self.r_ack * self.rend_ratio), abs(2 * self.r_ack * self.rend_ratio)))
            start_angle = -math.copysign(pi * 0.5, self.r_ack) + self.oT_w.rotation.yaw
            end_angle = start_angle + self.yaw_rate
            if self.yaw_rate < 0:
                pygame.draw.arc(self.py_map, (0, 0, 0), rect, end_angle, start_angle)
            else:
                pygame.draw.arc(self.py_map, (0, 0, 0), rect, start_angle, end_angle)

        # Draw trajectory history
        if len(self.trajectory_queue) != 0:
            traj = self.trajectory_queue[0:-1:10].copy()
            for point in traj:
                pygame.draw.circle(self.py_map, (50, 50, 50), self._sur_coord(point), 1)
        pygame.display.update()

    def _sur_coord(self, point):
        """
        transform to pygame surface coordinate
        :param point:
        :return:
        """
        if type(point) is tuple or type(point) is list:
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

    def reset_line(self, guideline_length=10, theta=None):
        assert guideline_length > 0, "Guide line length must be positive."
        if theta is None:
            theta = (self.oT_l.rotation.yaw + pi / 2) % (2 * pi)
        self.oT_l = Trans(Rot(theta), self.oT_nextnode.position)
        lT_nextnode = Trans(Rot(0), Vector2(guideline_length, 0))
        self.oT_nextnode = trans_from_mat(self.oT_l.as_mat() @ lT_nextnode.as_mat())


if __name__ == "__main__":
    tick = 100
    dt = 1 / tick
    wheel_sim = WheelTractionSim("/home/syeon/wheelchair_planner/simulator/settings.yaml")
    print("hello wheel chair")
    running = True
    clock = pygame.time.Clock()

    start = key_cur = time.time()
    v_r = 0
    phi = 0
    yaw_rate_command = 0
    i = 0
    keys = pygame.key.get_pressed()
    syeon_model = mpc.ackermann_mpc()
    while running:
        # keys = pygame.key.get_pressed()
        events = pygame.event.get()

        # if events[pygame.QUIT]:
        #     running = False
        # elif events[pygame.MOUSEBUTTONDOWN]:
        #     pass
        for event in events:
            if event.type == pygame.KEYDOWN:
                keys = pygame.key.get_pressed()
                if keys[pygame.K_SPACE]:
                    pass
                elif keys[pygame.K_q] or keys[pygame.K_ESCAPE]:
                    running = False
                elif keys[pygame.K_UP]:
                    print("increase robot velocity")
                    v_r += 0.1
                elif keys[pygame.K_DOWN]:
                    print("decrease robot velocity")
                    v_r -= 0.1
                elif keys[pygame.K_RIGHT]:
                    print("move to right")
                    phi -= 0.01
                elif keys[pygame.K_LEFT]:
                    print("move to left")
                    phi += 0.01
                elif keys[pygame.K_r]:
                    print("reset robot position")
                    wheel_sim.reset_line()
                elif keys[pygame.K_u]:
                    print("Undocking Wheelchair")
                    wheel_sim.docking(False)
                    v_r = 0
                    phi = 0
                    yaw_rate_command = 0
                elif keys[pygame.K_d]:
                    print("Docking Wheelchair")
                    wheel_sim.docking(True)
                    v_r = 0
                    phi = 0
                    yaw_rate_command = 0
                if not wheel_sim.with_chair:
                    if keys[pygame.K_k]:
                        print("turn to left")
                        yaw_rate_command += 0.1
                    elif keys[pygame.K_l]:
                        print("turn to right")
                        yaw_rate_command -= 0.1

        
        if wheel_sim.with_chair:
            # x_vel, y_vel, ang_vel = gen_from_rc(v_r, phi, wheel_sim.L)
            # MPC example
            syeon_model.state_update(wheel_sim.lT_r.position.y_val, wheel_sim.lT_r.rotation.yaw, sqrt(wheel_sim.x_vel**2 + wheel_sim.y_vel**2), wheel_sim.yaw_rate)
            syeon_model.ackermann_mpc()
            vel_command, steering_angle = syeon_model.for_simulation_command
            x_vel, y_vel, ang_vel = vel_command*np.cos(steering_angle), vel_command*np.sin(steering_angle), vel_command*np.sin(steering_angle)/syeon_model.l_wh
        else:
            x_vel = v_r
            y_vel = phi
            ang_vel = yaw_rate_command
        for i in range(5):
            wheel_sim.step(x_vel, y_vel, ang_vel)
        clock.tick(tick)
        cur = time.time()
        if (cur - key_cur) > 0.1:
            key_cur = cur
            key_flag = True
        i += 1
        if (cur - start) > 1:
            start = cur
            print(f"loop time : {i} Hz")
            i = 0
