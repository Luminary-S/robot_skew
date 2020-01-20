#!/usr/bin/python
# -*- coding: utf-8 -*-
#author:sgl
#date: 20200114

import rospy,time
import numpy as np
from frame_node import FrameNode

from .modules.ur_node import URNode  # relative import, should create __init__.py in the imported folder
from .modules.ur_move_st import Robot
from .modules.PID import PID
# from .modules.rcrController import RCRController


''' rcr controller'''
class RCRController(FrameNode):

    def __init__(self):
        super(RCRController, self).__init__()
        self.i =0
        self.rcr_num = 0
        self.height = 0
        self.vel = 0
        self.tar_vel = 0
        self.tar_height = 0
        self.status = "open"
        self.stop = 1
        self.rcr_up_times = 0
        self.rcr_down_times = 0

    def init_node(self, name='rcr_frame'):
        rospy.init_node(name, log_level=rospy.INFO)   

    def node_init(self):
        self.define_node_subscriber()

    # def define_node_publisher(self):
    #     rcr_pub = rospy.Publisher()

    def define_node_subscriber(self):
        rcr_sub = rospy.Subscriber('/rcr_data', rcr, self.callback_rcr,queue_size=1)

    def callback_rcr(self, msg):
        self.height = msg.ab_position
        self.vel = msg.velocity
        time.sleep(0.2)

    def get_height(self):
        return self.height
    def get_vel(self):
        return self.vel

    def set_tar_vel(self,vel):
        self.tar_vel=vel
    def set_tar_height(self,height):
        self.tar_height=height

    def rcr_move(self, h, v, direction):
        if direction == "down":
            self.rcrCtr.set_tar_height(-h)
            self.rcrCtr.set_tar_vel(v)
            self.rcrCtr.set_down()
        elif direction == "up":
            self.rcrCtr.set_tar_height(h)
            self.rcrCtr.set_tar_vel(v)
            self.rcrCtr.set_up()
        else:
            self.rcrCtr.set_stop()

    def set_stop(self):
        self.set_param("/rcr/stop/",1)
        self.set_param("/rcr/up", 0)
        self.set_param("/rcr/down", 0)
        self.set_param("/rcr/stepUp", 0)
        self.set_param("/rcr/stepDown/", 0)
    def set_clean(self):
        self.set_param("/rcr/clean/",1)
    def set_cleanstop(self):
        self.set_param("/rcr/cleanstop", 1)
    def set_pumpcleanrotate(self):
        self.set_param("/rcr/pumpcleanrotate/", 1)
    def set_pumpcleanstop(self):
        self.set_param("/rcr/pumpcleanstop", 1)
    def set_pumpsewagerotate(self):
        self.set_param("/rcr/pumpsewagerotate", 1)
    def set_pumpsewagestop(self):
        self.set_param("/rcr/pumpsewagestop/", 1)
    def set_stepUp(self):
        self.set_param("/rcr/stepUp", 1)
    def set_stepDown(self):
        self.set_param("/rcr/stepDown/", 1)
    def set_up(self):
        self.set_param("/rcr/up", 1)
        self.set_param("/rcr/tarHeight", self.tar_height)
        self.set_param("/rcr/tarVel", self.tar_vel)
    def set_down(self):
        self.set_param("/rcr/down", 1)
        self.set_param("/rcr/tarHeight", self.tar_height)
        self.set_param("/rcr/tarVel", self.tar_vel)

    def set_open_clean(self):
            # def set_clean(self):
        self.set_param("/rcr/clean/",1)
        self.set_param("/rcr/pumpcleanrotate/", 1)
        self.set_param("/rcr/pumpsewagerotate", 1)
    def set_stop_clean(self):
        self.set_param("/rcr/cleanstop", 1)
        self.set_param("/rcr/pumpcleanstop", 1)
        self.set_param("/rcr/pumpsewagestop/", 1)

    # def set_rcr_status(self,status):
    #     self.set_param("/rcr/status", status)

    def set_rcr_port(self, port, status):
        # set len & wid params
        try:
            self.set_param("/rcr/status", status)
            self.set_param("/rcr/port", port)
        except:
            self.logerr("set rcr port params error!")

    def __str__(self):
        return 'rcr Frame'

if __name__=="__main__":
    rcr = RCRController()
