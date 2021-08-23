import math
import sys

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from TimeManager import busy_delay


# ref : https://www.theconstructsim.com/ros-qa-053-how-to-move-a-robot-to-a-certain-point-using-twist/

class Mover:
    def __init__(self, _rate, Hz):
        self.x = -1.0
        self.y = -1.0
        self.theta = -1.0
        self.x_vel = -1.0
        self.z_vel = -1.0
        self.odom_stamp = None

        self.rate = _rate
        self.Hz = Hz

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber("/odom_combined", Odometry, self.newOdom, queue_size=1)

    def normalizeAngle(self, angle):
        while angle < 0:
            angle += 2 * math.pi
        while angle > 2 * math.pi:
            angle -= 2 * math.pi
        return angle

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z  # in radians

    def newOdom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.x_vel = msg.twist.twist.linear.x
        self.z_vel = msg.twist.twist.angular.z

        rot_q = msg.pose.pose.orientation
        _, _, th = self.euler_from_quaternion(rot_q.x, rot_q.y, rot_q.z, rot_q.w)
        self.theta = self.normalizeAngle(th)
        self.odom_stamp = msg.header.stamp.nsecs
        # print(self.x, self.y, (self.theta / math.pi) * 180)

    def get_theta(self):
        last = self.odom_stamp
        while self.odom_stamp == last:
            pass
        return self.theta

    def get_x(self):
        last = self.odom_stamp
        while self.odom_stamp == last:
            pass
        return self.x

    def get_y(self):
        last = self.odom_stamp
        while self.odom_stamp == last:
            pass
        return self.y


    def move_fwd(self, distance):

        x = self.get_x()
        y = self.get_y()

        goal_x = x + distance * math.cos(self.get_theta())
        goal_y = y + distance * math.sin(self.get_theta())

        diff_x = goal_x - x
        diff_y = goal_y - y

        while math.fabs(goal_x - self.get_x()) > 0.05 or math.fabs(goal_y - self.get_y()) > 0.05:
            if diff_x * (goal_x - self.get_x()) < 0 or diff_y * (goal_y - self.get_y()) < 0:
                print("over moving")
                break
            speed = Twist()
            speed.linear.x = 0.1
            speed.angular.z = 0.0

            self.vel_pub.publish(speed)
            busy_delay(0.1)

    def degree_diff(self, a, b): # return > 0 : a->b ccw
        a_x = math.cos(a)
        a_y = math.sin(a)

        b_x = math.cos(b)
        b_y = math.sin(b)

        if (a_x*b_x + a_y*b_y) - (-1) < sys.float_info.epsilon:
            return math.pi
        else:
            return math.acos(np.clip(a_x*b_x + a_y*b_y, -1.0, 1.0))

    def rotate_ccw(self, _phi):
        base = self.get_theta()

        speed = Twist()
        speed.linear.x = 0.0

        if _phi <= math.pi:
            speed.angular.z = math.pi / 12
            phi = _phi
        else:
            speed.angular.z = -math.pi / 12
            phi = -(_phi - 2*math.pi)

        while phi - self.degree_diff(base, self.get_theta()) > math.pi / 180:
            self.vel_pub.publish(speed)
            busy_delay(0.1)


