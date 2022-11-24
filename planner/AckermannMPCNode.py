import numpy as np
from math import atan2, sqrt, cos, sin
import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64MultiArray, Bool, MultiArrayLayout, MultiArrayDimension
from AckermannMPC import AckermannMPC


class MPCAckermannNode:
    def __init__(self, settings_yaml):
        self.mpc_model = AckermannMPC(settings_yaml)
        self.with_chair = self.mpc_model.with_chair

        rospy.init_node('ackermann_mpc_node', anonymous=True)
        # Subscriber
        # self.error_sub = rospy.Subscriber("error_from_line", PoseStamped, self.callback_error, queue_size=1)
        self.real_robot_sub = rospy.Subscriber("nav_to_planner", Float64MultiArray, self.callback_nav, queue_size=1)
        self.wheel_chair_docking_sub = rospy.Subscriber("is_dock", Bool, self.callback_with_chair, queue_size=1)
        self.is_single_mode_sub = rospy.Subscriber("is_single_mode", Bool, self.callback_is_single_mode, queue_size=1)
        self.is_dyn_exist_sub = rospy.Subscriber("is_dyn_exist", Bool, self.callback_is_dyn_exist, queue_size=1)
        # Publisher
        self.command_pub = rospy.Publisher("xytheta_vel", Float64MultiArray, queue_size=1)
        self.traj_pub = rospy.Publisher("local_path", Float64MultiArray, queue_size=1)

        # Timer
        self.control_timer = rospy.Timer(rospy.Duration(nsecs=int(0.01 * 1e+9)), self.control_callback, )

        self.control_seq = None
        self.t_ref_sec = None
        self.t_ref_nsec = None
        self.is_stop_by_dyn = False
        self.is_stop_by_nav = False
        self.prev_command_vector = np.array([0, 0, 0])
        self.prev_command_time = None

        rospy.loginfo("Start Ackermann MPC Node")

    def callback_is_dyn_exist(self, data: Bool):
        self.is_stop_by_dyn = data.data

    def callback_with_chair(self, data: Bool):
        self.with_chair = data.data

    def callback_is_single_mode(self, data: Bool):
        self.with_chair = not data.data

    def callback_nav(self, data: Float64MultiArray):
        # [t_0, y_err, theta_err, x_dot_r, y_dot_r, theta_dot_r, isStop]
        error: Float64MultiArray = data
        self.t_ref_sec, self.t_ref_nsec, t_cur, y_err, theta_err, x_dot_r, y_dot_r, theta_dot_r, self.is_stop_by_nav = error.data
        # print(f"error y : {error_y}, error_theta : {error_theta}, time : {error.header.stamp}")
        self.traj, self.control_seq = self.mpc_model.solve(y_err, theta_err, sqrt(x_dot_r ** 2 + y_dot_r ** 2),
                                                           theta_dot_r, t_cur,
                                                           self.with_chair)

        traj_msg = Float64MultiArray()
        traj_msg.data = self.traj.reshape((-1))
        traj_msg.layout = MultiArrayLayout(dim=[MultiArrayDimension(label="positions", size=self.traj.shape[0]),
                                                MultiArrayDimension(label="x_y_theta", size=self.traj.shape[1])],
                                           data_offset=0)
        self.traj_pub.publish(traj_msg)

    def control_callback(self, data: rospy.timer.TimerEvent):
        if self.control_seq is None:
            return
        # print(data.current_expected.to_sec())
        cur_time = data.current_expected.to_sec()
        cur_time = cur_time - (self.t_ref_sec + self.t_ref_nsec / 1e+9)
        # temp = self.control_seq[0, 0]
        if cur_time < self.control_seq[0, 0]:
            print(f'[control callback] cur_time is strange...{cur_time - self.control_seq[0, 0]}')
        elif cur_time > self.control_seq[-1, 0]:
            print('[control callback] cur_time exceed mpc prediction')
        # print(cur_time - self.control_seq[0, 0], data.current_expected.to_sec(), self.t_ref_sec , self.t_ref_nsec / 1e+9)

        is_stop = self.is_stop_by_dyn or self.is_stop_by_nav
        command_vector = Float64MultiArray()
        if not is_stop:
            com_vec = np.array([np.interp(cur_time, self.control_seq[:, 0], self.control_seq[:, 1]),
                                np.interp(cur_time, self.control_seq[:, 0], self.control_seq[:, 2]),
                                np.interp(cur_time, self.control_seq[:, 0], self.control_seq[:, 3])])
            command_vector.data = com_vec.reshape((-1))
            # print(self.control_seq[0, 0]-cur_time)

            # command_vector.x = self.control_seq[0,1]
            # command_vector.y = self.control_seq[0,2]
            # command_vector.z = self.control_seq[0,3]
        else:
            dx = self.prev_command_vector[0]
            dy = self.prev_command_vector[1]
            w = self.prev_command_vector[2]
            if w > 0:
                sign = 1
            else:
                sign = -1
            com_vec = np.array(
                [np.clip(dx - cos(atan2(dy, dx)) * self.mpc_model.acc_max * (cur_time - self.prev_command_time),
                         min(0, dx), max(0, dx)),
                 np.clip(dy - sin(atan2(dy, dx)) * self.mpc_model.acc_max * (cur_time - self.prev_command_time),
                         min(0, dy), max(0, dy)),
                 np.clip(w - sign*self.mpc_model.omega_acc_max * (cur_time - self.prev_command_time),
                         min(0, w), max(0, w))
                 ])
            # com_vec = np.array(
            #     [np.clip(dx - self.mpc_model.acc_max * (cur_time - self.prev_command_time),
            #              min(0, dx), max(0, dx)),
            #      np.clip(dy - self.mpc_model.acc_max * (cur_time - self.prev_command_time),
            #              min(0, dy), max(0, dy)),
            #      np.clip(w - self.mpc_model.omega_acc_max * (cur_time - self.prev_command_time),
            #              min(0, w), max(0, w))
            #      ])
            # com_vec = np.array([0, 0, 0])
            command_vector.data = com_vec.reshape((-1))

        self.command_pub.publish(command_vector)
        self.prev_command_vector = command_vector.data.copy()
        self.prev_command_time = cur_time


if __name__ == '__main__':
    yaml_dir = '../simulator/settings.yaml'
    mpc_planner = MPCAckermannNode(yaml_dir)
    rospy.spin()
