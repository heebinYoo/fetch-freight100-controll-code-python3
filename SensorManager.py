import rospy
from geometry_msgs.msg import Twist
import ros_numpy as rnp
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from Mover import Mover
import math
import random
import cv2
import CollisionDetactor


class SensorManager:
    def __init__(self):
        self.rgb_image = None
        self.depth_image = None
        self.laser_data = None
        self.ranges = None
        self.angle_increment = None
        self.angle_min = None
        self.angle_max = None

        self.rgb_stamp = None
        self.depth_stamp = None
        self.laser_stamp = None

    def rgb_callback(self, msg):
        self.rgb_image = cv2.cvtColor(rnp.numpify(msg), cv2.COLOR_BGR2RGB)
        self.rgb_stamp = msg.header.stamp.nsecs

    def depth_callback(self, msg):
        self.depth_image = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        self.depth_stamp = msg.header.stamp.nsecs

    def laser_callback(self, msg):
        # print(len(msg.ranges)) 662
        # print(msg.angle_min / math.pi * 180 ) -110
        # print(msg.angle_max / math.pi * 180) 110
        # print(msg.angle_increment / math.pi * 180) 0.3degree

        self.ranges = msg.ranges
        self.angle_increment = msg.angle_increment
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max

        raw_laser_data = np.array(msg.ranges[len(msg.ranges) // 2 - 25: len(msg.ranges) // 2 + 26])
        result = np.min(raw_laser_data[np.invert(np.isnan(raw_laser_data))])

        if not math.isinf(result):
            self.laser_data = result
        else:
            self.laser_data = -1
        self.laser_stamp = msg.header.stamp.nsecs

    def get_laser_dist(self):
        last = self.laser_stamp
        while self.laser_stamp == last:
            pass
        return self.laser_data

    def get_rgb(self):
        last = self.rgb_stamp
        while self.rgb_stamp == last:
            pass
        return self.rgb_image

    def get_depth(self):
        last = self.depth_stamp
        while self.depth_stamp == last:
            pass
        return self.depth_image

    def get_collision(self):
        result = False
        for _ in range(10):
            result = result or self.calc_collision()
        return result

    def calc_collision(self):
        last = self.laser_stamp
        while self.laser_stamp == last:
            pass

        unit = math.pi / 36
        k = int(unit / self.angle_increment)
        n = int(len(self.ranges) / k)
        shortest_distance_list = []
        for i in range(n):
            sliced = self.ranges[i * k:(i + 1) * k]
            sliced = np.array(sliced)
            if len(sliced[np.invert(np.isnan(sliced))]) == 0:
                shortest_distance_list.append(float('inf'))
            else:
                shortest_distance_list.append(np.min(sliced[np.invert(np.isnan(sliced))]))

        shortest_distance_list = np.array(shortest_distance_list)
        if self.angle_min < -math.pi / 2:
            left_redundant = int(math.fabs(self.angle_min + self.angle_increment * (15 / 2) - (-math.pi / 2)) / unit)
            shortest_distance_list = shortest_distance_list[left_redundant:]

        if self.angle_max > math.pi / 2:
            right_redundant = int(math.fabs(self.angle_max - self.angle_increment * (15 / 2) - math.pi / 2) / unit)
            shortest_distance_list = shortest_distance_list[:len(shortest_distance_list) - right_redundant]

        shortest_distance_list = shortest_distance_list[1:-1]
        for i, x in enumerate(shortest_distance_list):
            estimated = CollisionDetactor.calculate_collision_distance(-math.pi / 2 + (i + 1) * unit)
            if estimated > x:
                return True
        return False

