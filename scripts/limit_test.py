#!/usr/bin/env python

import rospy

import rosbag
import time
from threading import Lock

class wrenchRecorder():

    def __init__(self):
        self.lock = Lock()
        self.is_open = False

    def start(self):
        name_string = 'suctioncup_limit_'+ str(int(time.time())) + '.bag'
        rospy.loginfo("Recording bag with name: {0}".format(name_string))
        with self.lock:
            self.bag = rosbag.Bag (name_string, mode='w')
            self.is_open = True
            # TODO: find the topic names, and data type
            self.netft_subscriber = rospy.Subscriber('/netft/data',WrenchStamped,  self.force_torque_callback)

    def stop(self):
        rospy.loginfo("Stopping Recording")
        with self.lock:
            self.netft_subscriber.unregister()
            self.bag.close()
            self.is_open = False
        rospy.loginfo("Recording stopped.")

    def force_torque_callback(self,data):
        with self.lock:
            if self.is_open:
                self.bag.write('netft', data)

if __name__=='__main__':
    rec = wrenchRecorder()
    try:
        rec.start()
        rospy.loginfo("Press Ctrl + C to stop recording")
    except KeyboardInterrupt:
        rec.stop()
        return
