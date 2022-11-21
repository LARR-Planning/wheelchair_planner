import numpy as np
from math import atan2, sqrt
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
        # Publisher
        self.command_pub = rospy.Publisher("robot_vel_command", Vector3, queue_size=1)
        self.traj_pub = rospy.Publisher("local_path", Float64MultiArray, queue_size=1)

        # Timer
        self.control_timer = rospy.Timer(rospy.Duration(nsecs=int(0.01 * 1e+9)), self.control_callback)

        self.control_seq = None
        self.t_ref_sec = None
        self.t_ref_nsec = None

        rospy.loginfo("Start Ackermann MPC Node")

    def callback_with_chair(self, data: Bool):
        self.with_chair = data.data

    def callback_nav(self, data: Float64MultiArray):
        # [t_0, y_err, theta_err, x_dot_r, y_dot_r, theta_dot_r, isStop]
        error: Float64MultiArray = data
        self.t_ref_sec, self.t_ref_nsec, t_cur, y_err, theta_err, x_dot_r, y_dot_r, theta_dot_r, isStop = error.data
        # print(f"error y : {error_y}, error_theta : {error_theta}, time : {error.header.stamp}")
        self.mpc_model.state_update(y_err, theta_err, sqrt(x_dot_r ** 2 + y_dot_r ** 2), theta_dot_r, bool(isStop),
                                    self.with_chair)
        self.traj, self.control_seq = self.mpc_model.solve(t_cur)

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
        if cur_time < self.control_seq[0, 0] or cur_time > self.control_seq[-1, 0]:
            print('cur_time is strange...')
        # print(cur_time - self.control_seq[0, 0], data.current_expected.to_sec(), self.t_ref_sec , self.t_ref_nsec / 1e+9)
        command_vector = Vector3()
        command_vector.x = np.interp(cur_time, self.control_seq[:, 0], self.control_seq[:, 1])
        command_vector.y = np.interp(cur_time, self.control_seq[:, 0], self.control_seq[:, 2])
        command_vector.z = np.interp(cur_time, self.control_seq[:, 0], self.control_seq[:, 3])
        # print(self.control_seq[0, 0]-cur_time)

        # command_vector.x = self.control_seq[0,1]
        # command_vector.y = self.control_seq[0,2]
        # command_vector.z = self.control_seq[0,3]

        self.command_pub.publish(command_vector)


if __name__ == '__main__':
    yaml_dir = '../simulator/settings.yaml'
    mpc_planner = MPCAckermannNode(yaml_dir)
    rospy.spin()
