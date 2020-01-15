#!/usr/bin/python
# -*- coding: utf-8 -*-


class PID():

    def __init__(self, kp,ki,kd,kq=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kq = kq
        self.delta_val = 0.0
        self.pre_delta_val = .0
        self.sum_val = 0.0
        
        self.val = 0
        self.val_d = 0

    def set_pid_params(self, kp,ki,kd, kq=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kq = kq

    def update(self, val, val_d):
        self.val = val
        self.val_d = val_d

        self.pre_delta_val = self.delta_val
        self.delta_val = self.val_d - self.val

        self.diff_val = self.delta_val - self.pre_delta_val
        try:
            self.dao_diff_val = self.delta_val / self.diff_val 
            # if self.pre_delta_val == 0.0:
            #     self.dao_diff_val = 0.0
        except:
            self.dao_diff_val = -0.1

        self.sum_val = self.sum_val + self.delta_val
        if self.kq is not 0:
            print("kp:", self.kp * self.delta_val,",dao:",self.kq * self.dao_diff_val)

        u = self.kp * self.delta_val + self.ki * self.sum_val + self.kd * self.diff_val + self.kq * self.dao_diff_val
        return u

