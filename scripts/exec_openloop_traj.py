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
import csv
from std_msgs.msg import *
from geometry_msgs.msg import *
from threading import Lock

rospy.loginfo('Waiting for services...')
rospy.init_node('suctioncup_openloop', anonymous=True, disable_signals=True)
robot_ns = "/abb120"
rospy.wait_for_service(robot_ns + '/robot_SetWorkObject')

robot_SetSpeed = rospy.ServiceProxy(robot_ns + '/robot_SetSpeed', robot_SetSpeed)
robot_SetCartesian = rospy.ServiceProxy(robot_ns + '/robot_SetCartesian', robot_SetCartesian)
robot_GetCartesian = rospy.ServiceProxy(robot_ns + '/robot_GetCartesian', robot_GetCartesian)
rospy.loginfo('All services registered.')

pos = robot_GetCartesian()
p=[pos.x, pos.y, pos.z, pos.q0, pos.qx, pos.qy, pos.qz]
print('initial cartesian: ')
print(p)
robot_SetCartesian(*p)
robot_SetSpeed(20,20)

# read csv file, the robot end traj (need the transfer from rigid end to robot end!!!!)
filename = "robot_trajectory.csv"
a = np.loadtxt(open(filename , "rb"), delimiter=",")
traj = a.T
N = traj.shape[0]
p0 = p;
p0[0] = traj[0][0]
p0[1] = traj[0][1]
print("Please check the robot position:")
raw_input(p0)
robot_SetCartesian(*p0)
for i in range(N):
    pi = traj[i]
    print("Please check the robot position:")
    raw_input(pi)
    robot_SetCartesian(*pi)
    rospy.sleep(0.5)

print("Traj execution stopped")
