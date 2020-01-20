#!/usr/bin/python
# -*- coding: utf-8 -*-
#author:sgl
#date: 20200115

import rospy
import numpy as np
# from ur_node import URNode
from ur_move_st import Robot
from imu_node import IMUNode
from robot_core import *
# from frame_node import FrameNode

class Algorithm():
    def __init__(self):
        self.EKF = EKF()
        # self.ur = URNode()
        # self.imu = IMUNode()
        self.ur = Robot()

    def imp_part(self, F, Fd, Fi):
        """
        from the force sensor to give an impedance control value of  the EE added velocity
        e.g.:
        Fd = np.array([1,2,3,10,20,30]) # 1*6, format: force + torque
        stiffness_matrix = np.array([[1, 0, 0, 0, 0, 0],
                                    [0, 1, 0, 0, 0, 0],
                                    [0, 0, 1, 0, 0, 0],
                                    [0, 0, 0, 2, 0, 0],
                                    [0, 0, 0, 0, 3, 0],
                                    [0, 0, 0, 0, 0, 4],) # 6*6
        """
        stf_x = 0
        stf_y = 1333
        stf_z = 0
        stf_mx = 0
        stf_my = 0
        stf_mz = 0
        stf_mat = np.diag( [stf_x,stf_y,stf_z,stf_mx,stf_my,stf_mz]) #6*6
        delta_F = Fd - F
        Vf = np.exp( np.dot( delta_F, stf_mat) ) # 1*6
        return Vf
    
    def pos_part(self, q, Xd):
        """ get the position velocity added item"""
        Vp = np.zeros(6)
        Vp[2] = 0.01
        K = 0.01

        F_T = self.ur.FK(q)
        # R, X = TransToRp(F_T)        
        # F_T[][]
        R, p = TransToRp(F_T)
        # Xnow = p + np.array(p)
        Xnow = p
        delta_X = Xd - Xnow
        DX = np.concatenate( (delta_X, np.zeros(3)) )
        # F_Td = 
        Vd_b_e = Vp + K * DX

        return Vd_b_e
    
    def clean_part(self):
        Vp = np.zeros(6)
        Vp[2] = 0.01  # constant clean velocity
        # Vp 
        return Vp

    def vis_part(self, theta, img_d, img):
        c_z  = 0.3171 # depth : m
        k_image = 0.55 # gain of image error
        #########camera  information###########
        camera_inp = np.array([[1058.2, 0, 329.6861], [0, 1056.9, 172.0898], [0, 0, 1]])
        camera_ext = np.array([[0.0, -1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.35], [-1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        q = np.array(theta).copy()
        bTe  = self.ur.FK(q)
        bJe = self.ur.DK(q)   # ========
        bTc  = np.dot(bTe, camera_ext)
        # J_interaction = np.array(np.ones(2,6))
        ############ image controller ##############
        del_img = np.array(img_d) - np.array(img)
        Lmatrix = np.array([[-camera_inp[0, 0]/c_z, 0, u_/c_z], [0, -camera_inp[1, 1]/c_z, v_/c_z]])
        cVc = -1 * k_image * np.dot(np.linalg.pinv(Lmatrix), del_img)
        # d_error = np.sqrt(area) - d0
        # if abs(d_error) > 0.7 * d0:
        # d_error = 0.7 * d0
        # cVc(3) = cVc(3) - 1 * k_image * 0.0015 * d_error;

        # set boundary of the position cmd
        step_limitation = 0.05
        if np.linalg.norm(cVc) > step_limitation:
            cVc = cVc * step_limitation / np.linalg.norm(cVc)

        # robot control
        R,p = TransToRp(bTc)
        bVe = np.dot(R, cVc)
        dx = np.array([bVe[0], bVe[1], bVe[2], 0.0, 0.0, 0.0]) #  no rotation at head
        # dq = np.dot(np.linalg.pinv(bJe), dx)  =========
        # Vc = np.dot(del_img, J_interaction)
        return dx

    def imu_part(self, q, base_vel ):
        # imu velocity compensate item
        Vu = np.eye(4)
        # Q_ori, ang_vel_body, lin_acc_body = self.imu.get_data()
        # lin_vel, ang_vel = self.imu.update( Q_ori, ang_vel_body, lin_acc_body )
        # lin_vel = np.array(lin_vel)
        # ang_vel = np.array(ang_vel)
        F_T = self.ur.FK(q)
        Adj = Adjoint(F_T)
        # vb = np.concatenate( (ang_vel, lin_vel) )
        vb = np.array(base_vel).copy()
        Vu = -1.0 * np.dot( Adj, vb ) 
        return Vu
    
    def imu_data_filter(self, pre_data, now_data):
        new_lin_acc = self.EKF.low_filter_update( pre_data.lin_acc, now_data.lin_acc)
        pass

    def controller(self, Xlist, img, base_vel, F, q):
        # set window cleaning velocity
        V_clean = self.clean_part()
        # V_pos = self.pos_part()
        # set 
        V_imu = self.imu_part(q,base_vel)
        img_d = np.array([320,240]) # image center
        V_vis = self.vis_part(q, img_d, img)
        
        Fi = np.array([43.5,21.5,12.5,21.3,22,13])
        Fd = np.array([60,21.5,12.5,21.3,22,13])
        V_force = self.imp_part(F,Fd, Fi)


        V = V_clean + V_force
        bJe = self.ur.DK(q)
        dq = np.dot(np.linalg.pinv(bJe), V)
        # delta_x = self.pos_part(q, Xlist)
        # pass
        return dq
    
    def imp_controller(self, F,q):
        Fi = np.array([-3.37249994278,-1.58649992943,52.2024993896,0.0797999948263,1.27394998074,0.13014999032])
        Fd = np.array([60,21.5,12.5,21.3,22,13])
        V_force = self.imp_part(F,Fd, Fi)
        V = V_force
        bJe = self.ur.DK(q)
        dq = np.dot(np.linalg.pinv(bJe), V)
        return dq
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