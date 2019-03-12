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
from std_msgs.msg import Int32, String
from threading import Lock


class Calibration():

    def __init__(self):
        rospy.loginfo('Waiting for services...')
        self.lock = Lock()
        rospy.init_node('suctioncup_cal', anonymous=True, disable_signals=True)
        robot_ns = "/abb120" #TODO figure this out

        rospy.wait_for_service(robot_ns + '/robot_SetWorkObject')

        self.robot_SetSpeed = rospy.ServiceProxy(robot_ns + '/robot_SetSpeed', robot_SetSpeed)
        self.robot_SetCartesian = rospy.ServiceProxy(robot_ns + '/robot_SetCartesian', robot_SetCartesian)
        self.robot_GetCartesian = rospy.ServiceProxy(robot_ns + '/robot_GetCartesian', robot_GetCartesian)
        self.netft_subscriber = rospy.Subscriber('/netft/data', WrenchStamped, self.force_torque_callback)
        rospy.loginfo('All services registered.')

        # Initialize the robot
        self.robot_SetSpeed(50,50) #TODO: set robot speed to be slow

        pos = self.robot_GetCartesian()
        #[pos.x, pos.y, pos.z, pos.q0, pos.qx, pos.qy, pos.qz]
        print(pos)
        #self.robot_SetCartesian([pos.x, pos.y, pos.z, pos.q0, pos.qx, pos.qy, pos.qz])

    def force_torque_callback(self,data):
        with self.lock:
            self.cur_wrench = data

    def go_and_record(self, p, seqid):
        #TODO
        # set robot Cartesian
        self.robot_SetCartesian(p)
        #wait for a while
        rospy.sleep(1)
        pos = self.robot_GetCartesian()
        wrench = 0 #TODO: get wrench data
        time_now = rospy.Time.now()

        pos_data = to_poseStamped(time_now,seqid,[pos.x, pos.y, pos.z], [pos.q0, pos.qx, pos.qy, pos.qz])
        #wrench_data = to_wrenchStamped(time_now,seqid,[]); #TODO
        wrench_data = self.cur_wrench
        wrench_data.header.seq = seqid
        wrench_data.header.frame_id = 'ft_tool'

        # record the cartesian and wrench
        self.bag.write('position', pos_data)
        self.bag.write('wrench', wrench_data)

    def auto_calibration(self,init_cartesian, max_p, max_angle, iter):
        #given some p
        # go_and_record
        bagname = 'suctioncup_cal'+ str(int(time.time())) + '.bag'
        rospy.loginfo("Recording bag with name: {0}".format(bagname))
        self.bag = rosbag.Bag(bagname, mode='w')

        #TODO: get and set initial cartesian and wrench(close to zeros, need manually adjust)
        self.robot_SetCartesian(init_cartesian)
        self.init_wrench_data = self.cur_wrench
        self.init_cartesian = self.init_cartesian
        self.bag.write('initial position', init_cartesian)
        self.bag.write('initial wrench', self.init_wrench_data)

        R0 = tr.quaternion_matrix(init_cartesian[3:7])

        for i in range(iter):
            # compute p
            dp = [random.random(), random.random(), random.random()]
            dp = max_p*random.random()*dp/np.linalg.norm(dp)
            dr = [random.random(), random.random(), 0]
            dr = dr/np.linalg.norm(dr)
            dR = tr.rotation_matrix(max_angle*random.random(), dr)
            R = np.dot(R0,dR)
            quat_i = tr.quaternion_from_matrix(R)
            pos_i = dp + init_cartesian[0:3]
            rospy.loginfo("Please check the robot position:")
            raw_input(quat_i+pos_i)
            # go_and_record
            self.go_and_record(pos_i+quat_i, i)

        rospy.loginfo("Stopping Recording")
        self.bag.close()
        self.netft_subscriber.unregister()
        return

def to_poseStamped(t, seqid, p, q):
    poses = PoseStamped()
    poses.header.stamp = t
    poses.header.seq = seqid
    poses.header.frame_id = 'world'
    poses.pose.position.x = p[1]
    poses.pose.position.y = p[2]
    poses.pose.position.z = p[3]
    poses.pose.orientation.x = q[1]
    poses.pose.orientation.y = q[2]
    poses.pose.orientation.z = q[3]
    poses.pose.orientation.w = q[0]

    return poses

def to_wrenchStamped(t, seqid, f, tau):
    wrenchs = wrenchStamped()
    wrenchs.header.stamp = t
    wrenchs.header.seq = seqid
    wrenchs.header.frame_id = 'tool'
    wrenchs.wrench.force.x = f[1]
    wrenchs.wrench.force.y = f[2]
    wrenchs.wrench.force.z = f[3]
    wrenchs.wrench.torque.x = tau[1]
    wrenchs.wrench.torque.y = tau[2]
    wrenchs.wrench.torque.z = tau[3]

    return wrenchs

if __name__ == '__main__':
    cal = Calibration()
    rospy.sleep(1)
    init_cartesian = [] # TODO: find by experiment
    max_p = 0
    max_angle = 0
    iter = 10
    #cal.auto_calibration(init_cartesian, max_p, max_angle, iter)
