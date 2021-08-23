import time

import rospy
from geometry_msgs.msg import Twist
import ros_numpy as rnp
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from Mover import Mover
import math
import random
import cv2

from TimeManager import busy_delay
from SensorManager import SensorManager


class PatrolController:
    COLLECT_MODE = 1
    MOVE_MODE = 2

    def __init__(self, rate, _HERTZ):
        self.mode = self.COLLECT_MODE
        self.mover = Mover(rate, _HERTZ)
        self.node_list = []

    """
    레이저가, 자기 바로 앞 방향 말고 거의 180도 수집 가능하므로, 최적화 가능할듯 필요하면 2번만 수집해서 처리하도록
    """

    def collect(self):
        image_list = []
        laser_list = []
        depth_list = []
        for _ in range(360 // 15):
            image_list.append(sensorManager.get_rgb())
            laser_list.append(sensorManager.get_laser_dist())
            depth_list.append(sensorManager.get_depth())
            self.mover.rotate_ccw(_phi=math.pi / 12)
        self.node_list.append((image_list, laser_list, depth_list))

    def run(self):
        if patrolController.mode is self.COLLECT_MODE:
            self.collect()
            patrolController.mode = self.MOVE_MODE

        elif patrolController.mode is self.MOVE_MODE:

            # rotate
            """
            node_list[-1][1] 마지막으로 저장한 노드의, 두번째 원소 ~ 레이저 값 리스트
            self.node_list[-1][0][idx] # 미터만큼 멀리 떨어져있어.
            """

            # TODO, 어느 방향으로 이동할지에 대한 여러가지 전략이 있을 수 있겠음.
            #################################################################################
            idx = np.argmax(self.node_list[-1][1][2:23]) + 2
            print(idx)
            moving_distance = self.node_list[-1][1][idx] - 0.5
            print(moving_distance)
            if moving_distance < 0:
                print("can't move!!!")
                rospy.signal_shutdown("can't move forward, stop search")

            if random.random() < 0.3:
                r_idx = random.randrange(0, len(self.node_list[-1][1]) - 1)
                r_moving_distance = self.node_list[-1][0][idx] - 0.5
                if moving_distance > 0:
                    idx = r_idx
                    moving_distance = r_moving_distance
            #################################################################################

            self.mover.rotate_ccw(_phi=idx)
            # move
            for _ in range(int(moving_distance)):
                if self.collision_warn:
                    self.collision_warn = False
                    patrolController.mode = self.COLLECT_MODE
                    break

                self.mover.move_fwd(distance=1)
                self.collect()

            self.mover.move_fwd(distance=moving_distance % 1)

            patrolController.mode = self.COLLECT_MODE


def move_test():

    patrolController.mover.rotate_ccw(math.pi / 4)
    busy_delay(2)
    patrolController.mover.rotate_ccw(math.pi / 4)
    busy_delay(2)
    patrolController.mover.rotate_ccw(math.pi / 4)
    busy_delay(2)
    patrolController.mover.rotate_ccw(math.pi / 4)
    busy_delay(2)

    print("half")
    patrolController.mover.rotate_ccw(math.pi / 4)
    busy_delay(2)
    patrolController.mover.rotate_ccw(math.pi / 4)
    busy_delay(2)
    patrolController.mover.rotate_ccw(math.pi / 4)
    busy_delay(2)
    patrolController.mover.rotate_ccw(math.pi / 4)
    busy_delay(2)


    #patrolController.mover.move_fwd(1.0)
    #patrolController.mover.rotate(_phi=math.pi)
    #patrolController.mover.move_fwd(0.7)
    #patrolController.mover.rotate(_phi=math.pi)
    #patrolController.mover.move_fwd(0.7)
    #patrolController.mover.rotate(_phi=math.pi)


def sensor_test():
    img_warn = np.full((480, 640, 3), (0, 0, 255), np.uint8)
    img_false = np.zeros((480, 640, 3), np.uint8)


    while True:
        rgb_image = sensorManager.get_rgb()
        cv2.putText(rgb_image, "dist" + np.array2string(sensorManager.get_laser_dist()), (30, 30),
                    cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255))
        cv2.imshow("rgb", rgb_image)
        depth_image = sensorManager.get_depth()
        depth_image = np.nan_to_num(depth_image)
        cv2.imshow("depth", depth_image / np.max(depth_image))
        if sensorManager.get_collision():
            cv2.imshow("collision", img_warn)
        else:
            cv2.imshow("collision", img_false)
        cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node('fetch_controller')
    HERTZ = 10
    rate = rospy.Rate(HERTZ)
    sensorManager = SensorManager()
    rospy.Subscriber('/camera/rgb/image_rect_color', Image, sensorManager.rgb_callback)
    rospy.Subscriber('/base_scan', LaserScan, sensorManager.laser_callback)
    rospy.Subscriber('/camera/depth/image_rect', Image, sensorManager.depth_callback)

    patrolController = PatrolController(rate, HERTZ)
    move_test()

    #patrolController.collect()
    #    rospy.spin()
    while not rospy.is_shutdown():
        # patrolController.warn_require=True
        # print(patrolController.collision_warn)
        rate.sleep()
        pass
        # patrolController.node_list
        # TODO
