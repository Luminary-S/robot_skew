#!/usr/bin/python
# -*- coding: utf-8 -*-

# Copyright (c) 2019.12, Guangli SUN
# All rights reserved.

from robot_core import *
from ur_params import *
import numpy as np
np.set_printoptions(suppress=True, precision=5)

class Robot():

    def __init__(self):
        self.q0 = []
        self.q_init = []
        self.STEP = 0.005

    def set_init(self, M, Slist, N=6):
        # set the default init robot state , here we set 6 dof and all in 0 degree
        self.j_N = N
        q_0 = np.zeros((1,self.j_N))  # init for each joint is 0 
        self.Slist_0 = Slist 
        self.M = M
        self.q0 = q_0
        # Blist = []

    def set_UR_ROBOT(self, urtype):
        Glist,Mlist,Slist = UR_PARAMS(urtype)
        self.Slist_0 = Slist
        self.M = Mlist
        self.Glist = Glist
        return Glist,Mlist,Slist

    def omit_robot_info(self):
        print("robot is:")
        print("joint number:" + self.j_N)

    def POE(self, Theta, P, j_N=6):
        for i in range(j_N):
            # calculate screw coordinate of each joint
            theta = Theta[i]
            p = P[i]
            s = []

    def get_base_velocity_screw_axis(self, omega): 
        imu_pos = [0.215,0.325,]       
        omega_b = VecTose3(omega)
        vel_mat = [[1,0,0,0],
                    [0,1,0,0],
                    [0,0,1,0],
                    [0,0,0,1]]
        omega_b = np.array(vel_mat)
        return omega_b

    def FK(self, theta):
        Slist_0 = self.Slist_0
        M = self.M
        thetalist = np.array(theta).copy()
        T = FKinBody( M, Slist_0, thetalist )
        # R, p = TransToRp()
        return T
    
    def IK(self, q, Td):
        Slist_0 = self.Slist_0
        M = self.M
        q = np.array(q)
        eomg = 0.01
        ev = 0.001
        qd = IKinSpace(Slist_0, M, Td, q, eomg, ev)

    def DK(self, v, q):
        Slist_0 = self.Slist_0
        q = np.array(q)
        Js = JacobianSpace(Slist_0,q)
        Js_inv = np.linalg.pinv(Js)
        return np.dot(Js_inv,q)

    # def TtoR(self, T):

    def force_part(self, Fd):
        pass
    
    def ur_step_move(self, q , i, direction):
        # urk = self.kin
        F_T = self.FK(q)
        print("before F_t:", F_T)
        print("before up and down: ",F_T[0][3])
        # TransT = self.get_T_translation(F_T)
        print("========step=====", self.STEP)
        F_T[i][3] = F_T[i][3] + direction*self.STEP   

        qd = self.IK(q,F_T)
        T = self.FK(qd)
        print("up and down: ",T[0][3])
        return qd

    def ur_step_move_up(self,q):
        return self.ur_step_move(q,0,1)    
    def ur_step_move_down(self,q):
        return self.ur_step_move(q,0,-1)
    def ur_step_move_left(self,q):
        return self.ur_step_move(q,1,1)    
    def ur_step_move_right(self,q):
        return self.ur_step_move(q,1,-1)
    def ur_step_move_forward(self,q):
        return self.ur_step_move(q,2,1)    
    def ur_step_move_back(self,q):
        return self.ur_step_move(q,2,-1)

    def ur_xyz_move(self,q,xyz_list):
        #up:x; left:y; back:z
        [x,y,z] = xyz_list
        # urk = self.kin
        F_T = self.FK(q)
        F_T[2][3] = F_T[2][3] - x        
        F_T[0][3] = F_T[0][3] + y
        F_T[1][3] = F_T[1][3] - z
        # F_T[i] = F_T[i] + direction*self.STEP  
        qd = self.IK(q, F_T)
        return qd


    def spin_test(self):
        while 1:
            pass


if __name__ == "__main__":
    ur3 = Robot()
    Mlist = [[]]
    Slist = [[0,         0,         0,         0,        0,        0],
            [0,         1,         1,         1,        0,        1],
            [1,         0,         0,         0,       -1,        0],
            [0,   -0.1519,   -0.1519,   -0.1519, -0.11235, -0.06655],
            [0,         0,         0,         0,  0.4569,        0],
            [0,         0,     0.24365,   0.4569,        0,  0.4569]]  
    ur3.set_init(Mlist,Slist,6)
