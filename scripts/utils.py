import numpy as np
import tf.transformations as tr

def TransformWrench():
    transformed_wrench = []
    return transformed_wrench

def WrenchMapping(ref_cartesian, ref_wrench, ref_contact_wrench, wrench):
    return cartesian,contact_wrench


def addup_fixframe(p1,p2):
    # first p2, then p1
    quat = quat_tr2ros(tr.quaternion_multiply(\
    quat_ros2tr(p1[3:7]),quat_ros2tr(p2[3:7])))
    pos = p1[0:3] + p2[0:3]
    p = np.zeros(p1.shape)
    p[0:3] = pos
    p[3:7] = quat
    return p

def compute_increment(p1,p0):
    p = np.zeros(p1.shape)
    p[0:3] = p1[0:3] - p0[0:3]
    p[3:7] = quat_tr2ros(tr.quaternion_multiply(\
    quat_ros2tr(p1[3:7]),tr.quaternion_inverse(quat_ros2tr(p0[3:7]))))
    return p

def quat_tr2ros(q_tr):
    # q_tr: x y z w
    # q_ros: w x y z
    q_ros = np.zeros(q_tr.shape)
    q_ros[0]= q_tr[3]
    q_ros[1:] =  q_tr[0:3]]
    return q_ros

def quat_ros2tr(q_ros):
    q_tr = np.zeros(q_ros.shape)
    q_tr[3]= q_ros[0]
    q_tr[0:3] =  q_ros[1:]]
    return q_tr
