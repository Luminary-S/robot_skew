#!/usr/bin/python
# -*- coding: utf-8 -*-
# import sys
# sys.path.append("../")

### imu  pos estimation

# import yaml,os

# from PROJECTPATH import *

from frame_node import FrameNode

import rospy,time
# from ur3_kinematics import *
from std_msgs.msg import String
from sensor_msgs.msg import Imu
import numpy as np
import math
from robot_core import *

class IMUNode(FrameNode):
    def __init__(self):
        super(IMUNode, self).__init__()
        self.rate = 50
        self.lin_vel = np.zeros(3)
        self.ang_vel = np.zeros(3)
        self.sum_lin_vel = np.zeros(3)
        self.vel = np.concatenate(( self.ang_vel,self.lin_vel)) # omega, v
        self.pre_vel = self.vel
        # self.ur = UR(0)

    def init(self):
        self.init_node('imu_node')
        self.loginfo("start imu node...")
        self.define_node_publisher()
        self.define_node_subscriber()

    def define_node_subscriber(self):
        # img_sub = rospy.Subscriber('/baseCamImage', Image, self.callback_basecam, queue_size=1)
        imu_sub = rospy.Subscriber("/imu", Imu, self.callback_imu)
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
        lin_vel = self.lin_vel
        lin_acc = np.array(self.now_linear_acc)
        self.lin_vel = self.interger_lin_vel(lin_vel, lin_acc)
        self.ang_vel = np.array(self.now_angular_vel)
        self.pre_vel = self.vel
        self.vel = self.concate_ang_lin(self.ang_vel,self.lin_vel)

    def set_pre_data(self):
        self.pre_orientation = self.now_orientation
        self.pre_angular_vel = self.now_angular_vel
        self.pre_linear_acc = self.now_linear_acc

    def update(self, Q_ori, ang_vel_body, lin_acc_body ):
        # imu = self.EKF(imu)
        # base_attitude_qua = self.now_orientation
        lin_vel = 0
        ang_vel = 0
        print("linear vel:", lin_vel)
        print("ang vel:", ang_vel)
        return lin_vel, ang_vel

    def EKF(self, imu):
        return imu

    def get_R_2_w(self, Q_theta):
        return Q2r(Q_theta)

    def get_linear_pos(self, t, v, a):
        dt = 0.02
        t = t + v * dt + 1/2*a * dt * dt

    def get_data(self):
        Q_ori = self.now_orientation
        ang_vel_body = self.now_angular_vel
        lin_acc_body = self.now_linear_acc
        return  Q_ori, ang_vel_body, lin_acc_body
    
    def interger_lin_vel(self, lin_vel, lin_acc):
        lin_vel = lin_vel + lin_acc * 1.0 / self.rate
        return lin_vel
    def concate_ang_lin(self, lin,ang):
        return np.array((lin,ang))

    def spin_imu(self):
        rate = self.Rate(self.rate)
        time.sleep(2)
        while not self.is_shutdown():
            # imu = self.imu
            Q_ori, ang_vel_body, lin_acc_body = self.get_data()
            lin_vel, ang_vel = self.update( Q_ori, ang_vel_body, lin_acc_body )
            rate.sleep()

if __name__ == "__main__":
    imu = IMUNode()
    imu.init()
    imu.spin_imu()
