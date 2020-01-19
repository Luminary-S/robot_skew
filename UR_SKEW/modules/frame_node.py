#!/usr/bin/python
# -*- coding: utf-8 -*-
#author;sgl
#date: 20190805

import rospy

'''cam thread '''
class FrameNode(object):

    def __init__(self):
        self.num = 0

    def init(self):
        self.init_node('base_node')
        self.define_node_publisher()
        self.define_node_subscriber()

    def init_node(self, node_name):
        rospy.init_node(node_name, log_level=rospy.INFO)
        # img_sub = rospy.Subscriber('/baseCamImage', Image, self.callback_basecam, queue_size=1)
    
    def node_init(self):
        self.define_node_publisher()
        self.define_node_subscriber()

    def define_node_publisher(self):
        pass

    def define_node_subscriber(self):
        pass

    def set_param(self,param, val):
        rospy.set_param(param,val)
    def get_param(self,param):
        return rospy.get_param(param)

    def loginfo(self,info):
        rospy.loginfo(info)
    def logerr(self,info):
        rospy.logerr(info)
    def logwarn(self,info):
        rospy.logwarn(info)


    def set_sewage_param(self):
        pass

    def Rate(self,rate):
        return rospy.Rate(rate)

    def is_shutdown(self):
        return rospy.is_shutdown()

    def spin(self):
        pass

    def __str__(self):
        return 'base camera {}'.format(self.num)

