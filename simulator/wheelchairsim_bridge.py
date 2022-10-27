from WheelTractionSim import *
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Vector3, AccelStamped
from std_msgs.msg import Float32
import tf.transformations


class WCSimBase:
    def __init__(self, settings_yaml):
        rospy.init_node('wc_sim_node', anonymous=True)
        self.wheel_sim = WheelTractionSim(settings_yaml)
        self.cur_command = Vector3()  # x : x_vel, y : y_vel, z : yaw_rate

        # Subscriber
        self.command_sub = rospy.Subscriber("robot_vel_command", Vector3, self.callback_command)
        # Publisher
        self.odometry_pub = rospy.Publisher("robot_odom", Odometry, queue_size=3)
        self.acc_pub = rospy.Publisher("robot_acc", AccelStamped, queue_size=3)
        self.error_pub = rospy.Publisher("error_from_line", PoseStamped, queue_size=3)

        self.run_sim()

    def run_sim(self):
        r = rospy.Rate(1 / self.wheel_sim.dt)
        while not rospy.is_shutdown():
            self.update_sim()
            keys = pygame.key.get_pressed()
            if keys[pygame.K_q] or keys[pygame.K_ESCAPE]:
                pygame.quit()
                break
            r.sleep()

    def callback_command(self, data):
        command: Vector3 = data
        self.cur_command = command

    def update_sim(self):
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

        self.odometry_pub.publish(robot_odom)
        self.acc_pub.publish(robot_acc)
        self.error_pub.publish(error_from_line)

        self.wheel_sim.step(self.cur_command.x, self.cur_command.y, self.cur_command.z)


if __name__ == "__main__":
    wheel_sim_ros = WCSimBase("settings.yaml")
    # running = True
    # while running:
    #     keys = pygame.key.get_pressed()
    #     if keys[pygame.K_q] or keys[pygame.K_ESCAPE]:
    #         running = False
