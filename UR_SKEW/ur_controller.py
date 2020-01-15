#!/usr/bin/python
# -*- coding: utf-8 -*-
#author:sgl
#date: 20190805

import rospy
import numpy as np
from frame_node import FrameNode
from std_msgs.msg import String, Float32
from sensor_msgs.msg import JointState, Image
from demo_ur_skew.msg import rcr, sensorArduino, ft_sensor

from .modules.ur_node import URNode  # relative import, should create __init__.py in the imported folder
from .modules.ur_move_st import Robot
from .modules.PID import PID
# from .modules.rcrController import RCRController


class URController(FrameNode):
    def __init__(self):
        super(URController, self).__init__()
        self.ur = URNode()
        # self.rcrCtr = RCRController()
        self.status = "init"
        # self.init_variables()
        # self.camCtr = CAMController()  

    def init(self):
        self.init_node('ur_demo_node')
        self.loginfo("start UR demo control node...")
        self.define_node_publisher()
        self.define_node_subscriber()
    
    def define_node_publisher(self):
        self.joint_pub = rospy.Publisher("/ur_driver/URScript", String, queue_size=1)
        # self.vel_z_pub = rospy.Publisher("/vz", Float32, queue_size = 1)
        # self.vel_y_pub = rospy.Publisher("/vy", Float32, queue_size = 1)
        # self.vel_zd_pub = rospy.Publisher("/vzd", Float32, queue_size = 1)
        # self.vel_yd_pub = rospy.Publisher("/vyd", Float32, queue_size = 1)
        # self.Fzd_pub = rospy.Publisher("/Fzd", Float32, queue_size = 1)
    
    def define_node_subscriber(self):
        # img_sub = rospy.Subscriber('/baseCamImage', Image, self.callback_basecam, queue_size=1)
        joint_sub = rospy.Subscriber("/joint_states", JointState, self.callback_joint)
        # sensor_sub = rospy.Subscriber("/sensor_data", sensorArduino, self.callback_sensorAr, queue_size=1)
        # sensor_sub = rospy.Subscriber("/sensor_data", sensorArduino, self.callback_sensorAr ,queue_size=1)
        force_sub = rospy.Subscriber("/robotiq_ft_sensor", ft_sensor, self.callback_ft_sensor, queue_size=1)
        # imgPer_sub = rospy.Subscriber('/imgPercent', Float32, self.callback_percent)

    # standard callback function#
    def callback_sensorAr(self, msg):
        try:
            # print("sensor callback...")
            self.sonic = round(msg.sonic,2)  # 2 digit numbers
            # self.force = self.msg.force
        except:
            self.logerr("no sonic from sensor Arduino!")
    
    def callback_percent(self, msg):
        self.imgPer = msg.data

    def callback_joint(self, msg):
        self.read_pos_from_ur_joint(msg)
    
    def read_pos_from_ur_joint(self, msg):
        self.now_ur_pos = list(msg.position)
        self.now_vel = list(msg.velocity)
        self.ur.now_ur_pos = self.now_ur_pos
        self.ur.now_vel = self.now_vel

        # self.ur.set
    def callback_ft_sensor(self, msg):
        self.force = [msg.Fx, msg.Fy, msg.Fz, msg.Mx, msg.My, msg.Mz]  

    def cam_move(self, pitch, yaw):
        # self.camCtr.send_basecam_attitude_params(pitch, yaw)
        # self.camCtr.set_basecam_status()
        pass


class URStatusController():
    
    def __init__(self):
        print("IO part can be controller, through 1(start) or 0(stop)")

    def set_autoclean(self, status):
        rospy.set_param("/UR/autoclean", status)
    def set_finishclean(self, status):
        return rospy.set_param("/UR/finishclean",status)
    
    def get_finishclean(self):
        return rospy.get_param("/UR/finishclean")   

if __name__=="__main__":
    pass
