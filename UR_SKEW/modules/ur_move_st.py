#!/usr/bin/python
# -*- coding: utf-8 -*-

# Copyright (c) 2019.12, Guangli SUN
# All rights reserved.

from robot_core import *

import numpy as np
np.set_printoptions(suppress=True, precision=5)

class robot():

    def __init__(self):
        self.q0 = []
        self.q_init = []

    def set_init(self, N=6):
        # set the default init robot state , here we set 6 dof and all in 0 degree
        self.j_N = 6
        q_0 = np.zeros((1,j_N))  # init for each joint is 0 
        self.Slist_0 =[] 
        self.M = []
        self.q0 = q

    def omit_robot_info(self):
        print("robot is:")
        print("joint number:" + self.j_N)

    def POE(self, Theta, P, j_N=6):
        for i in range(j_N):
            # calculate screw coordinate of each joint
            theta = Theta[i]
            p = P[i]
            s = 

    def get_base_velocity_screw_axis(self, omega):        
        omega_b = VecTose3(omega)
        return omega_b

    def FK(self, theta):
        Slist_0 = self.Slist_0
        M_ee = self.M

    def TtoR(self, T):


    def vib_compensator(self):

        """
       dot( X_d )
        """