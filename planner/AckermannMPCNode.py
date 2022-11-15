import numpy as np
from math import atan2, sqrt
import rospy
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Float32MultiArray
from AckermannMPC_V3 import AckermannMPC


class MPCAckermannNode:
    def __init__(self, settings_yaml):
        self.mpc_model = AckermannMPC(settings_yaml)

        rospy.init_node('wc_sim_node', anonymous=True)
        # Subscriber
        # self.error_sub = rospy.Subscriber("error_from_line", PoseStamped, self.callback_error, queue_size=1)
        self.real_robot_sub = rospy.Subscriber("nav_topic", Float32MultiArray, self.callback_nav, queue_size=1)
        # Publisher
        self.command_pub = rospy.Publisher("robot_vel_command", Vector3, queue_size=1)

        self.x_vel_prev = 0
        self.y_vel_prev = 0

    def callback_nav(self, data):
        # [t_0, y_err, theta_err, x_dot_r, y_dot_r, theta_dot_r, isStop]
        error: Float32MultiArray = data
        _, y_err, theta_err, x_dot_r, y_dot_r, theta_dot_r, isStop = error.data
        # print(f"error y : {error_y}, error_theta : {error_theta}, time : {error.header.stamp}")
        self.mpc_model.state_update(y_err, theta_err, sqrt(x_dot_r**2+y_dot_r**2), theta_dot_r)
        vel_command, steering_angle = self.mpc_model.solve()[0]
        x_vel, y_vel, ang_vel = vel_command * np.cos(steering_angle), vel_command * np.sin(
            steering_angle), vel_command * np.sin(steering_angle) / self.mpc_model.l_wh

        if sqrt((self.x_vel_prev - x_vel)**2 + (self.y_vel_prev - y_vel)**2) > self.mpc_model.acc_max :
            print(f"Violation {sqrt((self.x_vel_prev - x_vel)**2 + (self.y_vel_prev - y_vel)**2) - self.mpc_model.acc_max}")
        self.x_vel_prev = x_vel
        self.y_vel_prev = y_vel

        command = Vector3()
        command.x = x_vel
        command.y = y_vel
        command.z = ang_vel
        self.command_pub.publish(command)


if __name__ == '__main__':
    mpc_planner = MPCAckermannNode('../simulator/settings.yaml')
    rospy.spin()
