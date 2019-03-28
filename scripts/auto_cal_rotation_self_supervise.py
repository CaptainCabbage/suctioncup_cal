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
GRIPPER_LENGTH = 165
RATE = 0.2857 # corresponding tangential translation rate to z change
Z_ref = 421.5
Z_low = 422
Z_high = 437
Z_reset = 445
delta_z = 1
delta_angle = 0.5
ITER = 5

#
RANDOM_SEED = 0

class RotationTest():

    def __init__(self):
        rospy.loginfo('Waiting for services...')
        self.lock = Lock()
        rospy.init_node('suctioncup_cal_rotation_test', anonymous=True, disable_signals=True)
        robot_ns = "/abb120"

        rospy.wait_for_service(robot_ns + '/robot_SetWorkObject')

        self.robot_SetSpeed = rospy.ServiceProxy(robot_ns + '/robot_SetSpeed', robot_SetSpeed)
        self.robot_SetCartesian = rospy.ServiceProxy(robot_ns + '/robot_SetCartesian', robot_SetCartesian)
        self.robot_GetCartesian = rospy.ServiceProxy(robot_ns + '/robot_GetCartesian', robot_GetCartesian)
        self.netft_subscriber = rospy.Subscriber('/netft/data', WrenchStamped, self.force_torque_callback)
        rospy.loginfo('All services registered.')

        bagname = 'suctioncup_cal_rotation_test'+ str(int(time.time())) + '.bag'
        print("Recording bag with name: {0}".format(bagname))
        self.bag = rosbag.Bag(bagname, mode='w')

        pos = self.robot_GetCartesian()
        p=[pos.x, pos.y, pos.z, pos.q0, pos.qx, pos.qy, pos.qz]
        print('initial cartesian: '),
        print(p)
        self.robot_SetCartesian(*p)
        self.init_cartesian = p
        self.reset_cartesian = [pos.x, pos.y, Z_reset, pos.q0, pos.qx, pos.qy, pos.qz]
        rospy.loginfo('Initialized.')
        np.random.seed(RANDOM_SEED)

    def test(self):

        print('Start rotation limit test.')
        iter = int(raw_input('Please enter the number of test: '))
        print('Total test time:'),#,end =" ")
        print(iter)

        R0 = tr.quaternion_matrix(self.init_cartesian[3:7])
        for i in range(iter):
            print('Test'),#,end =" ")
            print(i)
            Z_cur = float(raw_input('Please input your inital z (422 - 437): '))
            if Z_cur < 422:
                Z_cur = 422
            elif Z_cur > 437:
                Z_cur = 437

            p = self.init_cartesian
            p[2] = Z_cur
            #print('Press enter to confirm the position: ',p)
            #raw_input()
            self.go(p)

            dr = 2*(np.random.rand(1,2)-0.5)
            dr = np.append(dr/np.linalg.norm(dr),0)
            print('Randomly sampled rotational axis:'),#, end =" ")
            print(dr)
            angle = float(raw_input('Please input your desired rotational angle: '))
            print('Rotation angle: '),#, end =" ")
            print(angle)
            dt = float(raw_input('Please input your desired translational distance perpendicular to rotational axis: '))
            dp = dt*np.array([dr[1],-dr[0]])
            print('Tangential translation: '),#,end =" ")
            print(dt)
            dz = float(raw_input('Please input your desired dz: '))
            dp = np.append(dp,dz)

            dR = tr.rotation_matrix(angle*np.pi/180, dr)
            dp_compensate = np.array([0,0,-GRIPPER_LENGTH])-np.dot(dR[0:3,0:3],np.array([0,0,-GRIPPER_LENGTH]))
            R = np.dot(R0,dR)
            quat_i = tr.quaternion_from_matrix(R)
            pos_i = dp + dp_compensate + p[0:3]
            p_new = np.append(pos_i,quat_i)
            print("Check the robot position, press Enter to confirm: ")
            raw_input(p_new)
            self.go(p_new)
            raw_input('Press Enter to go back: ')
            self.go(p)
            flag = int(raw_input('Input 1 to record this configuration, 2 to reset: '))
            if flag == 1:
                self.go(p_new)
                self.record(i)
                self.go(p)
            elif flag == 2:
                self.go(self.reset_cartesian)
                self.go(p)
            else:
                print('You entered number other than 1,2 we will reset and this test will be discarded.')
                self.go(self.reset_cartesian)

        print('Test finished, stop recording.')
        self.bag.close()
        self.netft_subscriber.unregister()
        self.go(self.init_cartesian)
        return

    def self_supervise(self):
        angle_all = 2*np.ones([16])
        angle_all[0:3] = 1
        angle_all[11:16] = 3

        Z_cur = Z_low
        i = 0
        count = 0
        seqid = 0


        while Z_cur < Z_high:
            print('Current z:'),
            print(Z_cur)
            p = self.init_cartesian
            p[2] = Z_cur
            self.go(p)
            self.record(seqid)
            seqid = seqid + 1

            angle_max = angle_all[i]
            angle = delta_angle

            while angle < angle_max + 0.1:
                print('Rotation angle: '),
                print(angle_i)
                dt_random = np.sin(angle_i*np.pi/180)*np.random.random(ITER)
                dz_random = -np.sin(angle_i*np.pi/180)*np.random.random(ITER)
                for j in range(ITER):
                    dt = dt_random[j]
                    dz = dz_random[j]
                    dr = 2*(np.random.rand(1,2)-0.5)
                    dr = np.append(dr/np.linalg.norm(dr),0)
                    print('Randomly sampled rotational axis:'),#, end =" ")
                    print(dr)
                    print('Tangential translation: '),#,end =" ")
                    print(dt)
                    dp = np.append(dt*np.array([dr[1],-dr[0]]),dz)
                    dR = tr.rotation_matrix(angle*np.pi/180, dr)
                    dp_compensate = np.array([0,0,-GRIPPER_LENGTH])-np.dot(dR[0:3,0:3],np.array([0,0,-GRIPPER_LENGTH]))
                    R = np.dot(R0,dR)
                    quat_i = tr.quaternion_from_matrix(R)
                    pos_i = dp + dp_compensate + p[0:3]
                    p_new = np.append(pos_i,quat_i)
                    print("Check the robot position, press Enter to confirm: ")
                    raw_input(p_new)
                    wrench_0 = self.cur_wrench
                    self.go(p_new)
                    raw_input('Press Enter to go back: ')
                    self.go(p)

                    if cmp_netftdata(wrench_0, self.cur_wrench):
                        print('Good test, record data now.')
                        self.go(p_new)
                        self.record(seqid)
                        self.go(p)
                        seqid = seqid + 1
                        count = count + 1
                    else:
                        print('Bad test, reset vacuum gripper.')
                        self.go(self.reset_cartesian)
                        self.go(p)
                angle = angle + delta_angle

            Z_cur = Z_cur + delta_z
            i = i+1

        self.bag.close()
        print('Test finished, stop recording.')
        print('Total valid test: '),
        print(count)
        self.netft_subscriber.unregister()
        self.go(self.init_cartesian)
        return


    def go(self, p):
        print('Go to:'),#, end =" ")
        print(p)
        self.robot_SetCartesian(*p)
        rospy.sleep(5)

    def record(self,seqid):
        pos = self.robot_GetCartesian()
        time_now = rospy.Time.now()
        pos_data = to_poseStamped(time_now,seqid,[pos.x, pos.y, pos.z], [pos.q0, pos.qx, pos.qy, pos.qz])
        wrench_data = self.cur_wrench
        wrench_data.header.seq = seqid
        wrench_data.header.frame_id = 'ft_tool'

        # record the cartesian and wrench
        self.bag.write('position', pos_data)
        self.bag.write('wrench', wrench_data)
        print('Data recorded.')

    def force_torque_callback(self,data):
        with self.lock:
            self.cur_wrench = data

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

def cmp_netftdata(w0, w1):
    thr = np.array([0.2,0.2,0.2,0.2,0.2,0.2])

    d0 = np.array([w0.wrench.force.x,w0.wrench.force.y,w0.wrench.force.z,w0.wrench.torque.x,w0.wrench.torque.y,w0.wrench.torque.z])
    d1 = np.array([w1.wrench.force.x,w1.wrench.force.y,w1.wrench.force.z,w1.wrench.torque.x,w1.wrench.torque.y,w1.wrench.torque.z])
    d = np.abs(d0 - d1)
    r = d > thr

    if np.sum(r) > 0:
        return False
    else:
        return True


if __name__ == '__main__':
    rt = RotationTest()
    rospy.sleep(1)
    rt.self_supervise()
