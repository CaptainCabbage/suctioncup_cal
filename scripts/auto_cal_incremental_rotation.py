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
from std_msgs.msg import *
from geometry_msgs.msg import *
from threading import Lock

# vary among grippers, figure this out before experiments
GRIPPER_LENGTH = 135
RATE = 0.2857 # corresponding tangential translation rate to z change
Z_ref = 421.5
Z_low = 422
Z_high = 437

#
MAX_ANGLE = 2 # max random rotational angle, unit:degree
ITER_NUM = 5
RANDOM_SEED = 0

class Calibration():

    def __init__(self):
        rospy.loginfo('Waiting for services...')
        self.lock = Lock()
        rospy.init_node('suctioncup_cal_rotation', anonymous=True, disable_signals=True)
        robot_ns = "/abb120"

        rospy.wait_for_service(robot_ns + '/robot_SetWorkObject')

        self.robot_SetSpeed = rospy.ServiceProxy(robot_ns + '/robot_SetSpeed', robot_SetSpeed)
        self.robot_SetCartesian = rospy.ServiceProxy(robot_ns + '/robot_SetCartesian', robot_SetCartesian)
        self.robot_GetCartesian = rospy.ServiceProxy(robot_ns + '/robot_GetCartesian', robot_GetCartesian)
        self.netft_subscriber = rospy.Subscriber('/netft/data', WrenchStamped, self.force_torque_callback)
        rospy.loginfo('All services registered.')

        pos = self.robot_GetCartesian()
        p=[pos.x, pos.y, pos.z, pos.q0, pos.qx, pos.qy, pos.qz]
        print('initial cartesian: ')
        print(p)
        self.robot_SetCartesian(*p)
	rospy.loginfo('Initialized.')

    def force_torque_callback(self,data):
        with self.lock:
            self.cur_wrench = data

    def go_and_record(self, p, seqid):
        #TODO
        # set robot Cartesian
        print('Go')
        self.robot_SetCartesian(*p)
        #wait for a while
        rospy.sleep(5)
        pos = self.robot_GetCartesian()
        time_now = rospy.Time.now()
        print('recording data...')
        pos_data = to_poseStamped(time_now,seqid,[pos.x, pos.y, pos.z], [pos.q0, pos.qx, pos.qy, pos.qz])
        wrench_data = self.cur_wrench
        wrench_data.header.seq = seqid
        wrench_data.header.frame_id = 'ft_tool'

        # record the cartesian and wrench
        self.bag.write('position', pos_data)
        self.bag.write('wrench', wrench_data)

    def auto_calibration(self,init_cartesian, max_p_rate, max_angle, iter):
        #given some p
        # go_and_record
        print('Start auto-calibration.')
        bagname = 'suctioncup_cal_rotation'+ str(int(time.time())) + '.bag'
        print("Recording bag with name: {0}".format(bagname))
        self.bag = rosbag.Bag(bagname, mode='w')

        #TODO: get and set initial cartesian and wrench(close to zeros, need manually adjust)
        self.robot_SetCartesian(*init_cartesian)
        self.init_wrench_data = self.cur_wrench
        self.init_wrench_data.header.seq = 0
        self.init_wrench_data.header.frame_id = 'ft_tool'
        self.init_cartesian = init_cartesian
        self.bag.write('position',  to_poseStamped(rospy.Time.now(), 0,init_cartesian[0:3], init_cartesian[3:7]))
        self.bag.write('wrench', self.init_wrench_data)

        R0 = tr.quaternion_matrix(init_cartesian[3:7])

        np.random.seed(RANDOM_SEED)

        dz = (Z_high - Z_low)*np.random.rand(iter)
        dt = 2*(np.random.rand(iter,2)-0.5)
        dt_v = np.random.rand(iter)*max_p_rate*(dz + Z_low - Z_ref)

        angle_all = np.random.rand(iter)
        dr_all = np.random.rand(iter,2)

	    # go back to initial position every time
        print('Test with going back to intial position every trial:')
	    # for rotation only
        for i in range(iter):
            # compute p
            dp = np.array([0,0,0])
            dr = np.append(dr_all[i],0)
            dr = dr/np.linalg.norm(dr)
            angle = max_angle*angle_all[i]
            print('rotation angle: ')
            print(angle)
            dR = tr.rotation_matrix(angle*np.pi/180, dr)
            dp_compensate = np.array([0,0,-GRIPPER_LENGTH])-np.dot(dR[0:3,0:3],np.array([0,0,-GRIPPER_LENGTH]))
            #print(dR)
            R = np.dot(R0,dR)
            quat_i = tr.quaternion_from_matrix(R)
            pos_i = dp + dp_compensate + init_cartesian[0:3]
            print("Please check the robot position:")
            p_new = np.append(pos_i,quat_i)
            raw_input(p_new)
            # go_and_record
            self.go_and_record(p_new, i+i+1)
            self.robot_SetCartesian(*init_cartesian)
            rospy.sleep(5)

        print('Test with not going back to intial position every trial:')
        raw_input('Press Enter to Continue:')

	    # for rotation only
        for i in range(iter):
            # compute p
            dp = np.array([0,0,0])
            dr = np.append(dr_all[i],0)
            dr = dr/np.linalg.norm(dr)
            angle = max_angle*angle_all[i]
            print('rotation angle: ')
            print(angle)
            dR = tr.rotation_matrix(angle*np.pi/180, dr)
            dp_compensate = np.array([0,0,-GRIPPER_LENGTH])-np.dot(dR[0:3,0:3],np.array([0,0,-GRIPPER_LENGTH]))
            #print(dR)
            R = np.dot(R0,dR)
            quat_i = tr.quaternion_from_matrix(R)
            pos_i = dp + dp_compensate + init_cartesian[0:3]
            print("Please check the robot position:")
            p_new = np.append(pos_i,quat_i)
            raw_input(p_new)
            # go_and_record
            self.go_and_record(p_new, i+i+1)

        self.robot_SetCartesian(*init_cartesian)

        print("Stopping Recording")
        self.bag.close()
        self.netft_subscriber.unregister()
        return

def to_poseStamped(t, seqid, p, q):
    poses = PoseStamped()
    poses.header.stamp = t
    poses.header.seq = seqid
    poses.header.frame_id = 'world'
    poses.pose.position.x = p[0]
    poses.pose.position.y = p[1]
    poses.pose.position.z = p[2]
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
    pos = cal.robot_GetCartesian()
    p=[pos.x, pos.y, pos.z, pos.q0, pos.qx, pos.qy, pos.qz]
    init_cartesian = p # TODO: find by experiment
    cal.auto_calibration(init_cartesian, RATE, MAX_ANGLE, ITER_NUM)
