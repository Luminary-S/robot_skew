#!/usr/bin/python
# -*- coding: utf-8 -*-
#author:sgl
#date: 20200115

import rospy
import numpy as np
from .modules.ur_node import URNode
# from frame_node import FrameNode

class algorithm():
    def __init__(self):
        self.EKF = EKF()
        self.ur = URNode()

    def force_part(self, F, Fd, Fi):
        """
        from the force sensor to give an impedance control value of the designed the velocity of
        e.g.:
        Fd = np.arrayr([1,2,3,10,20,30]) # 1*6, format: force + torque
        stiffness_matrix = np.array([[1, 0, 0, 0, 0, 0],
                                    [0, 1, 0, 0, 0, 0],
                                    [0, 0, 1, 0, 0, 0],
                                    [0, 0, 0, 2, 0, 0],
                                    [0, 0, 0, 0, 3, 0],
                                    [0, 0, 0, 0, 0, 4],) # 6*6
        """
        stf_x = 1
        stf_y = 1
        stf_z = 1
        stf_mx = 1
        stf_my = 2
        stf_mz = 3
        stf_mat = np.diag( [stf_x,stf_y,stf_z,stf_mx,stf_my,stf_mz]) #6*6
        delta_F = Fd - F
        Vf = np.exp( np.dot( delta_F, stf_mat) ) # 1*6
        return Vf
    
    def pos_part(self, q, Xd):
        Vp = np.zeros(6)
        Vp[1] = 0.05
        K = 0.01

        X = self.ur.FK(q)        

        Vd_b_e = Vp + K * ( Xd - X )

        return Vd_b_e

    def vis_part(self, img_d, img):
        extrinc_param = np.array(np.ones(2,6)
        del_img = np.array(img_d) - np.array(img)
        V_ee = np.dot(del_img, extrinc_param)
        return Vi

    def imu_part(self, imu_d):
        Vu = np.eye(4)
        return Vu
    
    def imu_data_filter(self, pre_data, now_data):
        new_lin_acc = self.EKF.low_filter_update( pre_data.lin_acc, now_data.lin_acc)
        pass

    def controller(self, X_d, img_d, imu_d, Fd):
        pass

class EKF():
    def __init__(self):
        print("extened kalman filter of the sensor date")

    def low_filter_update(self, pre_data, now_data):
        pre_data = np.array(pre_data)
        size = pre_data.shape[0]
        new_data = pre_data + np.dot( pre_data , ( np.identity(size)* 0.8 ) )
        return new_data

    def EKF(self):
        pass