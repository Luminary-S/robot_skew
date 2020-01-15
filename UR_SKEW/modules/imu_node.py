#!/usr/bin/python
# -*- coding: utf-8 -*-
# import sys
# sys.path.append("../")

### imu  pos estimation

import yaml,os

# from PROJECTPATH import *

from frame_node import FrameNode

import rospy,time
# from ur3_kinematics import *
from std_msgs.msg import String
from sensor_msgs.msg import JointState, Imu

import math

class IMUNode(FrameNode):
    def __init__(self):
        super(IMUNode, self).__init__()
        self.rate = 50
        # self.ur = UR(0)

    def init(self):
        self.init_node('imu_node')
        self.loginfo("start imu node...")
        self.define_node_publisher()
        self.define_node_subscriber()

    def define_node_subscriber(self):
        # img_sub = rospy.Subscriber('/baseCamImage', Image, self.callback_basecam, queue_size=1)
        imu_sub = rospy.Subscriber("/imu_data", Imu, self.callback_imu)
        # sensor_sub = rospy.Subscriber("/sensor_data", sensorArduino, self.callback_sensorAr, queue_size=1)

    def define_node_publisher(self):
        self.joint_pub = rospy.Publisher("/ur_driver/URScript", String, queue_size=10)

    def callback_imu(self, msg):
        self.read_pos_from_imu(msg)

    def read_pos_from_imu(self, msg):
        self.set_pre_data()
        self.now_orientation = list( msg.orientation )
        self.now_angular_vel = list( msg.angular_velocity )
        self.now_linear_acc = list( msg.linear_acceleration )

    def set_pre_data(self):
        self.pre_orientation = self.now_orientation
        self.pre_angular_vel = self.now_angular_vel
        self.pre_linear_acc = self.now_linear_acc

    def update(self, imu):
        imu = self.EKF(imu)
        base_attitude_qua = self.now_orientation
        imu_next = []
        return imu_next

    def EKF(self, imu):
        return imu


    def get_linear_pos(self, t, v, a):
        dt = 0.02
        t = t + v * dt + 1/2*a * dt * dt
