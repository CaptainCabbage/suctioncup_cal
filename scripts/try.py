#!/usr/bin/env python2.7
import rospy
from robot_comm.msg import robot_CartesianLog
from robot_comm.srv import *
import random
import math
import numpy as np
#import tf.transformations as tr
import rosbag
import time
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped

rospy.init_node('suctioncup_cal', anonymous=True, disable_signals=True)

bagname = '/home/xianyi/suctioncup_cal'+ str(int(time.time())) + '.bag'
bag = rosbag.Bag(bagname, mode='w')

pose = PoseStamped()
pose.header.stamp = rospy.Time.now()
pose.header.frame_id = '1'
pose.pose.position.x = 0.1
pose.pose.position.y = 0.2
pose.pose.position.z = 0.25

pose.pose.orientation.x = 1
pose.pose.orientation.y = 3
pose.pose.orientation.z = 2
pose.pose.orientation.w = 4

bag.write('initial_position', pose)

bag.close()

