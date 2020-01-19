#!/usr/bin/python
# -*- coding: utf-8 -*-
# import sys
# sys.path.append("../")

import yaml,os

# from PROJECTPATH import *

from frame_node import FrameNode

import rospy,time
# from ur3_kinematics import *
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from demo_ur_skew.msg import rcr, sensorArduino, ft_sensor
# from jacobian import *
# from ur_move import UR
import math
from ur_move_st import Robot


class URNode(FrameNode):
    def __init__(self):
        super(URNode, self).__init__()
        self.rate = 50
        self.ur = Robot()
        # self.ur = UR(0)

    def init(self):
        self.init_node('ur_node')
        self.loginfo("start UR node...")
        self.define_node_publisher()
        self.define_node_subscriber()

    # def node_init(self):
    #     self.define_node_publisher()
    #     self.define_node_subscriber()

    def define_node_subscriber(self):
        # img_sub = rospy.Subscriber('/baseCamImage', Image, self.callback_basecam, queue_size=1)
        joint_sub = rospy.Subscriber("/joint_states", JointState, self.callback_joint)
        # sensor_sub = rospy.Subscriber("/sensor_data", sensorArduino, self.callback_sensorAr, queue_size=1)
        force_sub = rospy.Subscriber("/robotiq_ft_sensor", ft_sensor, self.callback_ft_sensor, queue_size=1)

    def callback_ft_sensor(self, msg):
        self.force = [msg.Fx, msg.Fy, msg.Fz, msg.Mx, msg.My, msg.Mz] 

    def define_node_publisher(self):
        self.joint_pub = rospy.Publisher("/ur_driver/URScript", String, queue_size=10)

    def callback_joint(self, msg):
        self.read_pos_from_ur_joint(msg)

    def read_pos_from_ur_joint(self, msg):
        self.now_ur_pos = list(msg.position)
        self.now_vel = list(msg.velocity)

    def get_ur_rosparams(self):
        self.urtype = self.get_param("/UR/type")
        self.status = self.get_param("/UR/status")
        self.rate = self.get_param("/UR/rate")

    def get_rosparams(self):
        self.get_ur_rosparams()
    
    def set_ur_rosparams(self):
        self.set_param("/UR/type",self.urtype)
        self.set_param("/UR/status",self.status)
        self.set_param("/UR/rate",self.rate)

    def set_ur(self):
        self.ur.set_UR_ROBOT(self.urtype)

    """ element ur move """
    # q should be in rad unit, if not use getpi(q) to transfer it
    def ur_move_to_point(self, pub, q ):
        str = self.buildMove(self.moveType, q)
        self.sendMove(pub, str)

    #for test
    def ur_movej_to_point(self, pub, q ):
        t,vel,ace = self.set_ur_pub_params()
        t = 0
        vel = 0.3
        ace = 0.2
        # q_in = self.getpi(q)
        self.set_moveType("fld")
        self.ur_move_to_point(self.joint_pub,q)

    def ur_movejp_to_point(self, pub, q ):
        t,vel,ace = self.set_ur_pub_params()
        t = 0
        vel = 0.3
        ace = 0.2
        # q_in = self.getpi(q)
        self.set_moveType("flp")
        self.ur_move_to_point(self.joint_pub,q)
        # self.urscript_pub_movel( pub, q, vel, ace, t )
        # self.urscript_pub( pub, q_in, vel, ace, t )

    def set_ur_pub_params(self):
        self.t = 0
        self.vel = 0.05
        self.ace = 0.2
        return self.t, self.vel, self.ace

    def set_moveType(self, str):
        self.moveType = str

    def buildMove(self, moveType="stop", pose = [83.84, -153.38, 23.84, 115.12, -102.87, 357.59 ]):
        # Moving on jointpositions = "movej([-1.95, -1.58, 1.16, -1.15, -1.55, 1.18], a=1.0, v=0.1)"+"\n"
        # Moving on posepositions = "movej(p[0.00, 0.3, 0.4, 2.22, -2.22, 0.00], a=1.0, v=0.1)" + "\n"
        acceleration = 1.0  # Joint acceleration in rad/s^2
        speed = 0.5
        time = 0  # Time the move must take
        radius = 0  # Blend radius in m, so the robot moves trough the point instead of stopping
        array = list(pose)
        if moveType[0] == "f":
            acceleration = 0.5  # Joint acceleration in rad/s^2
            speed = 0.1  # Joint speed in rad/s
        elif moveType[0] == "s":
            acceleration = 0.05  # Joint acceleration in rad/s^2
            speed = 0.05  # Joint speed in rad/s

        moveType = moveType[1:]
        if moveType == "j" or moveType == "l":
            sendable = "move%s(%s, a=%s, v=%s, t=%s, r=%s)" % (moveType, array, acceleration, speed, time, radius)
        elif moveType == "jp" or moveType == "lp":
            sendable = "move%s(p%s, a=%s, v=%s, t=%s, r=%s)" % (moveType[0], array, acceleration, speed, time, radius)
        elif moveType == "jd" or moveType == "ld":
            sendable = "move%s([%s], a=%s, v=%s, t=%s, r=%s)" % (moveType[0], ",".join([ "d2r("+str(i)+")" for i in array]), acceleration, speed, time, radius)
        else:
            sendable = "stopj[1.2]"
        # sendable = "movej(p[0.00, 0.3, 0.4, 2.22, -2.22, 0.00], a=1.0, v=0.1"
        # print sendable
        return sendable

    def ur_stop(self):
        pub = self.joint_pub
        self.sendMove( pub, "stopj[1.2]" )

    def sendMove(self, pub, string):
        st = String()
        st.data = string
        pub.publish(st)
        print("Published", string)
    def urscript_pub(self, pub, qq, vel, ace, t):
        ss = "movej([" + str(qq[0]) + "," + str(qq[1]) + "," + str(qq[2]) + "," + str(qq[3]) + "," + str(qq[4]) + "," + str(qq[5]) + "]," + "a=" + str(ace) + "," + "v=" + str(vel) + "," + "t=" + str(t) + ")"
        print("---------ss:", ss)
        # ss="movej([-0.09577000000000001, -1.7111255555555556, 0.7485411111111111, 0.9948566666666667, 1.330836666666667, 2.3684322222222223], a=1.0, v=1.0,t=5)"
        pub.publish(ss)

    def urscript_speedj_pub(self, pub , vel, ace, t):
        ss = "speedj([" + str(vel[0]) + "," + str(vel[1]) + "," + str(vel[2]) + "," + str(vel[3]) + "," + str(
            vel[4]) + "," + str(vel[5]) + "]," + "a=" + str(ace) + "," + "t=" + str(t) + ")"
        # print("---------ss:", ss)
        # ss="movej([-0.09577000000000001, -1.7111255555555556, 0.7485411111111111, 0.9948566666666667, 1.330836666666667, 2.3684322222222223], a=1.0, v=1.0,t=5)"
        pub.publish(ss)
    
    def getpi(self, l):
        res = []
        print(l)
        for i in l:
            res.append( i / 180 * math.pi )
        return res
    
    def getDegree(self, l):
        res = []
        for i in l:
            res.append(  i*180 / math.pi )
        return res
    
    # def spin(self):
    #     rate = self.Rate(self.rate)
    #     pos0 = [0.17, -78.88, 49.33, -149.68, -89.77, 268.29]
    #     pos1 =  [ -0.30, -119.21, 119.79, -178.76, -89.86, 267.45]
    #     pos = pos1
    #     while not self.is_shutdown():
    #         self.get_rosparams()
    #         self.set_moveType("sjd")
            
    #         self.ur_move_to_point(self.joint_pub, pos)
    #         pos = pos0
    #         rate.sleep()

    def ik_test(self, status):
        pub = self.joint_pub
        # p1 = []
        # self.ur_move_to_point(pub, p1)
        direction_list = [ "up", "down", "back", "forward", "left", "right", "start", "stop", "init" ]
        direction = direction_list[status]
        print("direction:",direction)
        # len = 0.04
        q = self.now_ur_pos
        if direction == "start":
            self.set_moveType("sjd")
            self.ur_move_to_point(pub, [-47.723,-52.315,115.869,-56.231,54.547,256.017])
        elif direction  == "down":
            qd = self.ur.ur_step_move_down(q)
            self.ur_movej_to_point(pub,qd)
        elif direction == "up": 
            print("enter up...")
            # q0 =[-47.723,-52.315,115.869,-56.231,54.547,256.017]
            # q0 = self.getpi(q0)
            # print("init pos:", self.ur.FK(q0))
            # q = self.ur.get_q()
            qd = self.ur.ur_step_move_up(q)
            # print("qd",qd)
            # self.ur_movej_to_point(pub,qd)
            # qd = self.ur.ur_step_move_up(q)
        elif direction == "back": 
            qd = self.ur.ur_step_move_backward(q)
            self.ur_movej_to_point(pub,qd)
        elif direction == "forward": 
            qd = self.ur.ur_step_move_forward(q)
            self.ur_movej_to_point(pub,qd)
        elif direction == "right": 
            qd = self.ur.ur_step_move_right(q)
            self.ur_movej_to_point(pub,qd)
        elif direction == "left": 
            qd = self.ur.ur_step_move_left(q)
            self.ur_movej_to_point(pub,qd)
        elif direction == "stop":
            self.ur_stop()
    
    def spin_ik_test(self):
        rate = self.Rate(self.rate)
        while not self.is_shutdown():
            # self.get_rosparams()    
            try:
                inp = input("direction? [0,1,2,3,4,5,6,7], [up, down, back, forward, left, right, init, stop]: ")  #in python3ï¼Œ no function raw_input,input gets string
                # if inp == "q":
                #     break
                self.ik_test(int(inp))
            except KeyboardInterrupt:
                rospy.signal_shutdown("KeyboardInterrupt")
                raise
            rate.sleep()

    

if __name__=="__main__":
    ur_robot = URNode()
    ur_robot.init()
    ur_robot.ur.set_UR_ROBOT("ur3")
    ur_robot.spin_ik_test()