import sys
import time
from random import random

sys.path.append('../')
from simulator.utils import *
from simulator.WheelTractionSim import WheelTractionSim
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Vector3, AccelStamped
from std_msgs.msg import Float64MultiArray, Bool


class WCSimBase:
    def __init__(self, settings_yaml):
        rospy.init_node('wc_sim_node', anonymous=False)
        # rospy.rostime.switch_to_wallclock()
        self.wheel_sim = WheelTractionSim(settings_yaml)
        self.cur_command = Float64MultiArray()  # x : x_vel, y : y_vel, z : yaw_rate
        self.cur_command.data = [0, 0, 0]

        self.is_stop = False

        # Subscriber
        self.command_sub = rospy.Subscriber("xytheta_vel", Float64MultiArray, self.command_callback, queue_size=1)
        self.pred_traj_sub = rospy.Subscriber("local_path", Float64MultiArray, self.pred_traj_callback, queue_size=1)
        # Publisher
        self.docking_pub = rospy.Publisher("is_dock", Bool, queue_size=1)
        self.odometry_pub = rospy.Publisher("robot_odom", Odometry, queue_size=1)
        self.acc_pub = rospy.Publisher("robot_acc", AccelStamped, queue_size=1)
        self.error_pub = rospy.Publisher("error_from_line", PoseStamped, queue_size=1)
        self.real_robot_pub = rospy.Publisher("nav_to_planner", Float64MultiArray, queue_size=1)

        self.t_ref_sec = int(rospy.rostime.get_rostime().to_sec())
        self.t_ref_nano = int(rospy.rostime.get_rostime().to_nsec() % 1e+9)

        # Timer
        self.sim_timer = rospy.Timer(rospy.Duration(nsecs=int(self.wheel_sim.dt * 1e+9)), self.sim_timer_callback)
        rospy.spin()

        # self.run_sim()

    def sim_timer_callback(self, data):
        events = pygame.event.get()
        for event in events:
            if event.type == pygame.KEYDOWN:
                keys = pygame.key.get_pressed()
                if keys[pygame.K_q] or keys[pygame.K_ESCAPE]:
                    pygame.quit()
                    rospy.signal_shutdown("Quit simulation")
                    break
                elif keys[pygame.K_l]:
                    print("reset robot position")
                    # theta = pi * random.random()
                    ######
                    # Enter the next guide line's angle HERE!
                    theta = self.wheel_sim.oT_l.rotation.yaw + pi * 2 / 3
                    # wheel_sim.oT_l.rotation.yaw : Current guide line's angle
                    # -pi/2 : turn right w.r.t. current guide line
                    ######
                    self.wheel_sim.reset_line(theta)
                elif keys[pygame.K_r]:
                    print("reset robot position")
                    # theta = pi * random.random()
                    ######
                    # Enter the next guide line's angle HERE!
                    theta = self.wheel_sim.oT_l.rotation.yaw - pi * 2 / 3
                    # wheel_sim.oT_l.rotation.yaw : Current guide line's angle
                    # -pi/2 : turn right w.r.t. current guide line
                    ######
                    self.wheel_sim.reset_line(theta)
                elif keys[pygame.K_s]:
                    self.is_stop = not self.is_stop
                    print("[INFO]Stop Command Received" if self.is_stop else "[INFO]!GO GO GO!")
                elif keys[pygame.K_u]:
                    print("Undocking Wheelchair")
                    self.wheel_sim.docking(False)
                    is_docking = Bool()
                    is_docking.data = False
                    for _ in range(10):
                        self.docking_pub.publish(is_docking)
                elif keys[pygame.K_d]:
                    print("Docking Wheelchair")
                    self.wheel_sim.docking(True)
                    is_docking = Bool()
                    is_docking.data = True
                    for _ in range(10):
                        self.docking_pub.publish(is_docking)
        self.update_sim(data)

    def command_callback(self, data):
        command: Float64MultiArray = data
        self.cur_command = command

    def pred_traj_callback(self, data: Float64MultiArray):
        self.wheel_sim.pred_traj = np.array(data.data).reshape(data.layout.dim[0].size, data.layout.dim[1].size)

    def update_sim(self, data: rospy.timer.TimerEvent):
        self.wheel_sim.step(self.cur_command.data[0], self.cur_command.data[1], self.cur_command.data[2],
                            exec_time=data.current_expected.to_sec())
        # sim_time = rospy.Time(self.wheel_sim.sim_time)
        sim_time = data.current_expected.to_sec() - (self.t_ref_sec + self.t_ref_nano / 1e+9)
        # print(sim_time)
        # get odometry
        robot_odom = Odometry()
        robot_odom.pose.pose.position.x = self.wheel_sim.oT_r.position.x_val
        robot_odom.pose.pose.position.y = self.wheel_sim.oT_r.position.y_val
        quat_w = cos(self.wheel_sim.oT_r.rotation.yaw * 0.5)
        quat_z = sin(self.wheel_sim.oT_r.rotation.yaw * 0.5)
        robot_odom.pose.pose.orientation.w = quat_w
        robot_odom.pose.pose.orientation.z = quat_z
        robot_odom.twist.twist.linear.x = self.wheel_sim.robot_vel.position.x_val
        robot_odom.twist.twist.linear.y = self.wheel_sim.robot_vel.position.y_val
        robot_odom.twist.twist.angular.z = self.wheel_sim.robot_vel.rotation.yaw
        robot_odom.header.stamp = sim_time
        robot_odom.header.frame_id = 'map'

        # get acceleration
        robot_acc = AccelStamped()
        robot_acc.accel.linear.x = self.wheel_sim.robot_acc.position.x_val
        robot_acc.accel.linear.y = self.wheel_sim.robot_acc.position.y_val
        robot_acc.accel.angular.z = self.wheel_sim.robot_acc.rotation.yaw
        robot_acc.header.stamp = sim_time

        # get error from line
        error_from_line = PoseStamped()
        error_from_line.pose.position.y = self.wheel_sim.lT_r.position.y_val
        err_quat_w = cos(self.wheel_sim.lT_r.rotation.yaw * 0.5)
        err_quat_z = sin(self.wheel_sim.lT_r.rotation.yaw * 0.5)
        error_from_line.pose.orientation.w = err_quat_w
        error_from_line.pose.orientation.z = err_quat_z
        error_from_line.header.stamp = sim_time

        # publish array like topic
        # [t_0, y_err, theta_err, x_dot_r, y_dot_r, theta_dot_r, isStop]
        real_robot_topic = Float64MultiArray()

        real_robot_topic.data = [self.t_ref_sec,
                                 self.t_ref_nano,
                                 sim_time,  # time
                                 self.wheel_sim.lT_r.position.y_val,  # y_err
                                 self.wheel_sim.lT_r.rotation.yaw,  # theta err
                                 self.wheel_sim.robot_vel.position.x_val,  # x_dot_r
                                 self.wheel_sim.robot_vel.position.y_val,  # y_dot_r
                                 self.wheel_sim.robot_vel.rotation.yaw,  # theta_dot_r
                                 self.is_stop
                                 ]

        self.odometry_pub.publish(robot_odom)
        self.acc_pub.publish(robot_acc)
        self.error_pub.publish(error_from_line)
        self.real_robot_pub.publish(real_robot_topic)


if __name__ == "__main__":
    wheel_sim_ros = WCSimBase("settings.yaml")
