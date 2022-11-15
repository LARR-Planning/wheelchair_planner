from WheelTractionSim import *
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Vector3, AccelStamped
from std_msgs.msg import Float32, Float32MultiArray
import tf.transformations


class WCSimBase:
    def __init__(self, settings_yaml):
        rospy.init_node('wc_sim_node', anonymous=True)
        self.wheel_sim = WheelTractionSim(settings_yaml)
        self.cur_command = Vector3()  # x : x_vel, y : y_vel, z : yaw_rate

        self.is_stop = False

        # Subscriber
        self.command_sub = rospy.Subscriber("robot_vel_command", Vector3, self.callback_command)
        # Publisher
        self.odometry_pub = rospy.Publisher("robot_odom", Odometry, queue_size=3)
        self.acc_pub = rospy.Publisher("robot_acc", AccelStamped, queue_size=3)
        self.error_pub = rospy.Publisher("error_from_line", PoseStamped, queue_size=3)
        self.real_robot_pub = rospy.Publisher("nav_topic", Float32MultiArray, queue_size=3)

        self.run_sim()

    def run_sim(self):
        r = rospy.Rate(1 / self.wheel_sim.dt)
        d_theta = [pi * 2 / 3, -pi * 2 / 3, -pi * 2 / 3, pi / 2, -pi / 2]
        theta_inx = 0
        while not rospy.is_shutdown():
            events = pygame.event.get()
            for event in events:
                if event.type == pygame.KEYDOWN:
                    keys = pygame.key.get_pressed()
                    if keys[pygame.K_q] or keys[pygame.K_ESCAPE]:
                        pygame.quit()
                        break
                    elif keys[pygame.K_r]:
                        print("[INFO]Reset robot position")
                        # theta = pi * random.random()
                        ######
                        # Enter the next guide line's angle HERE!
                        theta = self.wheel_sim.oT_l.rotation.yaw + d_theta[theta_inx % len(d_theta)]
                        # wheel_sim.oT_l.rotation.yaw : Current guide line's angle
                        # -pi/2 : turn right w.r.t. current guide line
                        ######
                        self.wheel_sim.reset_line(theta)
                    elif keys[pygame.K_s]:
                        self.is_stop = not self.is_stop
                        print("[INFO]Stop Command Received" if self.is_stop else "[INFO]!GO GO GO!")
            self.update_sim()
            r.sleep()

    def callback_command(self, data):
        command: Vector3 = data
        self.cur_command = command

    def update_sim(self):
        self.wheel_sim.step(self.cur_command.x, self.cur_command.y, self.cur_command.z)

        sim_time = rospy.Time(self.wheel_sim.sim_time)
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
        real_robot_topic = Float32MultiArray()

        real_robot_topic.data = [self.wheel_sim.sim_time,  # time
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
    # running = True
    # while running:
    #     keys = pygame.key.get_pressed()
    #     if keys[pygame.K_q] or keys[pygame.K_ESCAPE]:
    #         running = False
