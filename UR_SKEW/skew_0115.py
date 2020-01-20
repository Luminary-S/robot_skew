#!/usr/bin/python
# -*- coding: utf-8 -*-
# import sys
# sys.path.append("../")

import yaml,os
import rospy,time
import sys
sys.path.append(".")
# import package

from std_msgs.msg import String, Float32
# import cv2
# from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import JointState, Image

from frame_node import FrameNode
import numpy as np

# from . import modules.IMUNode
from modules.kin_algorithm import Algorithm
from modules.imu_node import IMUNode
from modules.ur_node import URNode


class demoController(FrameNode):
    def __init__(self):
        super(demoController, self).__init__()
        self.rate = 30
        # self.ur_Ctr = URController()
        # self.rcr_Ctr = RCRController()
        self.ur = URNode()
        self.alg = Algorithm()
        self.imu = IMUNode()
        # self.camCtr = CAMController()

    def init(self):
        self.init_node("demo_200114_node")
        self.loginfo("start demo 200114 node...")
        self.ur.node_init()
        self.imu.node_init()
        self.init_variables()
        # self.ctr.node_init()  
        
    def init_variables(self):
        self.rcr_times = 0
        self.clean_times = 0
        self.status = 'init'
        self.clean_status = 'init' 

    def update(self,status):
        # status = self.status
        clean_times = 3
        pub = self.ur.joint_pub
        if status is "init":
            self.loginfo("===== 1. init stage =====")
            q_init = [-47.728,-52.309,115.85,-56.205,54.543,255.993]
            self.ur.ur_movej_to_point(pub, q_init)
            rospy.sleep(5)
            status = "start"
        elif status is "start":
            self.loginfo("===== 2. start cleaning stage =====")
            flag = self.start()
            if flag is True:
                status = "touch"
        elif status is "touch":
            self.loginfo("===== 3. touch stage =====")
            flag = self.touch()
            if flag is True:
                status = "touch"
            else:
                status = "init"
        elif status is "clean":
            self.loginfo("===== 4."+ str(self.clean_times)+" cleaning  stage =====" )
            flag = self.clean()
            if flag is False:
                status = "right"
                self.set_finishclean(1)
            # else:
                self.clean_times += 1
            if self.clean_times >= clean_times:
                if self.get_finishclean():
                    self.clean_status = 'stop'
                # self.status = "init"
                    status = "init"
            # else:
        elif status is "right":
            self.loginfo("===== 5."+ str(self.clean_times)+" moving stage =====" )
            self.right()
            rospy.sleep(3)
            status = "clean"
        return status
    
    def start(self):
        pub = self.ur.joint_pub
        q0 = [-46.582,-33.003,112.162,-71.84,55.664,256.173]
        self.ur.ur_movej_to_point(pub, q0)
        rospy.sleep(3)
        return True
    
    def touch(self):
        pub = self.ur.joint_pub
        # Xlist = np.array([0,0.2,0])
        # img = np.array([200,100]) # not use, if use change to feature point
        # base_vel = self.imu.get_data()
        F = np.array(self.ur.force)
        q = np.array(self.ur.now_ur_pos)
        dq = self.alg.imp_controller(F, q)
        if np.linalg.norm(dq) > 0.001:
            t = 0
            vel = 0.1
            ace = 0.1
            self.ur.urscript_speedj_pub(pub,vel, ace, t)
            return True
        else:
            print("touch!!!")
            return False
    
    def right(self):
        self.set_finishclean(1)
        pub = self.ur.joint_pub
        q = np.array(self.ur.now_ur_pos)
        xyz_list = [0.2,0.04, 0.01]
        qd = self.ur.ur.ur_xyz_move(q,xyz_list)
        self.ur.ur_movejp_to_point(pub,qd)
        # self.ur.moveType("")

    def clean(self):
        self.set_autoclean(1)
        pub = self.ur.joint_pub
        Xlist = np.array([0,0.2,0])
        img = np.array([200,100]) # not use, if use change to feature point
        base_vel = self.imu.get_data()
        F = np.array(self.ur.force)
        q = np.array(self.ur.now_ur_pos)
        dq = self.alg.controller(Xlist, img, base_vel, F, q)
        if np.linalg.norm(dq) > 0.001:
            t = 0
            vel = 0.3
            ace = 0.2
            self.ur.urscript_speedj_pub(pub,vel, ace, t)
            return True
        else:
            print("clean stop!")
            return False

    def set_autoclean(self, status):
        self.set_param("/UR/autoclean", status)
    def set_finishclean(self, status):
        return self.set_param("/UR/finishclean",status)    
    def get_finishclean(self):
        return self.get_param("/UR/finishclean")

    def spin(self):
        rate = self.Rate(self.rate)
        time.sleep(8)
        status = "init"
        while not self.is_shutdown():
            # self.get_rosparams()
            # if self.rcr_times > 2 and self.clean_status == "stop":
            #     status = 'stop'
            # i = i + 1
            status = self.update(status)
            self.status = status
            # if 
            rate.sleep()

if __name__=="__main__":
    ur_robot = demoController()
    ur_robot.init()
    ur_robot.spin()