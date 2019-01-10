#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import rospkg
import numpy as np
import cv2

from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError

class DataSaver(object):

    def __init__(self):
        self.camera_sub = rospy.Subscriber("/camera_image", Image, self.cameraCallback, queue_size=10)
        self.lidar_sub  = rospy.Subscriber("/lidar_image", Image, self.lidarCallback, queue_size=10)
        self.flag_sub   = rospy.Subscriber("/save_flag", Bool, self.flagCallback, queue_size=1)
        self.flag = False

        self.rospack = rospkg.RosPack()
        self.path = self.rospack.get_path('depthimage_creater')

        self.count = 0

    def cameraCallback(self, image_msg):
        try:
            self.camera_image = CvBridge().imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
            print (e)

    def lidarCallback(self, image_msg):
        try:
            self.lidar_image = CvBridge().imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
            print (e)

    def flagCallback(self, bool_msg):
        if bool_msg.data:
            self.flag = True

    def saver(self):
        print(self.path)

        camera_path = self.path + "/data/calibration/camera_" + str(self.count) + ".jpg"
        lidar_path  = self.path + "/data/calibration/lidar_" + str(self.count) + ".jpg"

        print(camera_path)
        print(lidar_path)

        self.count += 1

    def main(self):
        rospy.init_node("data_saver")
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.flag:
                self.saver()
                self.flag = False
            else:
                print("wait Flag")
            rate.sleep()

        return 0

if __name__ == '__main__':
    print("start")
    ds = DataSaver()
    ds.main()
