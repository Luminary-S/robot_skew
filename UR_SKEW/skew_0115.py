#!/usr/bin/python
# -*- coding: utf-8 -*-
# import sys
# sys.path.append("../")

import yaml,os
import rospy,time

from std_msgs.msg import String, Float32
# import cv2
# from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import JointState, Image
from demo_ur_skew.msg import rcr, sensorArduino, ft_sensor
from frame_node import FrameNode

from .modules.ur_node import URNode
from rcr_controller import RCRController
from ur_controller import URController
import numpy as np

class demoController(FrameNode):
    def __init__(self):
        super(demoController, self).__init__()
        self.rate = 30
        self.ur_Ctr = URController()
        self.rcr_Ctr = RCRController()
        self.ur = URNode()
        # self.camCtr = CAMController()

    def init(self):
        self.init_node('demo_200114 node ')
        self.loginfo("start demo_200114 node...")
        self.ur.node_init()
        self.init_variables()
        # self.ctr.node_init()  
        
    def init_variables(self):
        self.rcr_times = 0
        self.clean_times = 0
        self.status = 'init'
        self.clean_status = 'init' 

    def update(self):
        # status = self.status
        pass
    
    def path(self):
        pass

    def spin(self):
        rate = self.Rate(self.rate)
        time.sleep(8)
        while not self.is_shutdown():
            # self.get_rosparams()
            if self.rcr_times > 2 and self.clean_status == "stop":
                status = 'stop'
            # i = i + 1
            self.update()
            # if 
            rate.sleep()

if __name__=="__main__":
    ur_robot = demoController()
    ur_robot.init()
    ur_robot.spin()