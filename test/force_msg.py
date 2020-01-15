#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy,time
import numpy as np
from demo_singlercr.msg import rcr

rospy.init_node('rcr_test')
rospy.loginfo("start sensor_arduino node...")
pub = rospy.Publisher("/rcr", rcr, queue_size = 1)
rate = rospy.Rate(30)
time.sleep(0)
rcrmsg = rcr()

while not rospy.is_shutdown():
    rcrmsg.velocity = 3.0 
    rcrmsg.ab_position = 3.3
    pub.publish(rcrmsg)
    rate.sleep()


