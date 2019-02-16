#!/usr/bin/env python

import rospy
from robot_comm.msg import robot_CartesianLog
from robot_comm.srv import *
import random
import math
import numpy as np
import tf.transformations as tr
import rosbag
import time
from std_msgs.msg import Float64MultiArray

class Calibration():

    def __init__(self):
        rospy.loginfo('Waiting for services...')
        rospy.init_node('suctioncup_cal', anonymous=True, disable_signals=True)
        robot_ns = "/abb120/robot_node"

        rospy.wait_for_service(robot_ns + '/robot_SetWorkObject')

        # TODO: add needed, delete useless
        #robot_SetWorkObject = rospy.ServiceProxy(robot_ns + '/robot_SetWorkObject', robot_SetWorkObject)
        #robot_SetTool = rospy.ServiceProxy(robot_ns + '/robot_SetTool', robot_SetTool)
        #robot_SetZone = rospy.ServiceProxy(robot_ns + '/robot_SetZone', robot_SetZone)
        self.robot_SetSpeed = rospy.ServiceProxy(robot_ns + '/robot_SetSpeed', robot_SetSpeed)
        self.robot_SetCartesian = rospy.ServiceProxy(robot_ns + '/robot_SetCartesian', robot_SetCartesian)
        self.robot_GetCartesian = rospy.ServiceProxy(robot_ns + '/robot_GetCartesian', robot_GetCartesian)
        #robot_GetIK = rospy.ServiceProxy(robot_ns + '/robot_GetIK', robot_GetIK)
        #robot_SetJoints = rospy.ServiceProxy(robot_ns + '/robot_SetJoints', robot_SetJoints)
        rospy.loginfo('All services registered.')

        bagname = 'suctioncup_cal'+ str(int(time.time())) + '.bag'
        self.bag = rosbag.Bag(bagname, mode='w')

        #TODO: get initial cartesian and wrench(close to zeros, need manually adjust)
        self.init_pos_data = 0
        self.init_wrench_data = 0
        self.bag.write('initial position', pos_data)
        self.bag.write('initial wrench', wrench_data)

        #TODO: set robot speed to be slow

        #pos = self.robot_GetCartesian()
        #[pos.x, pos.y, pos.z, pos.q0, pos.qx, pos.qy, pos.qz]
        #robot_SetCartesian(workspace_center[0], workspace_center[1], workspace_center[2], *initial_quaternion)


    def go_and_record(self, p):
        #TODO
        # set robot Cartesian
        self.robot_SetCartesian(p)
        #wait for a while
        rospy.sleep(1)
        pos = self.robot_GetCartesian()
        wrench = 0 #TODO
        pos_data = [pos.x, pos.y, pos.z, pos.q0, pos.qx, pos.qy, pos.qz]
        wrench_data = []; #TODO
        # TODO: get wrench data


        # record the cartesian and wrench
        self.bag.write('position', pos_data)
        self.bag.write('wrench', wrench_data)


    def limit_test(self):
        #TODO:
        # go in six directions to and record the forces
        # get all the feasible forces in the convex hull of sampled forces

    def auto_calibration(self):




def to_poseStamped(t, frameid, p, q):
    poses = PoseStamped()
    poses.header.stamp = t
    poses.header.frame_id = frameid
    poses.pose.position.x = p[1]
    poses.pose.position.y = p[2]
    poses.pose.position.z = p[3]
    poses.pose.orientation.x = q[1]
    poses.pose.orientation.y = q[2]
    poses.pose.orientation.z = q[3]
    poses.pose.orientation.w = q[0]

    return poses

def to_wrenchStamped(t, frameid, f, tau):
    wrenchs = wrenchStamped()
    wrenchs.header.stamp = t
    wrenchs.header.frame_id = frameid
    wrenchs.wrench.force.x = f[1]
    wrenchs.wrench.force.y = f[2]
    wrenchs.wrench.force.z = f[3]
    wrenchs.wrench.torque.x = tau[1]
    wrenchs.wrench.torque.y = tau[2]
    wrenchs.wrench.torque.z = tau[3]

    return wrenchs
