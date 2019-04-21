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
    q_ros = np.zeros(4)
    q_ros[0]= q_tr[3]
    q_ros[1:] =  q_tr[0:3]
    return q_ros

def quat_ros2tr(q_ros):
    q_tr = np.zeros(4)
    q_tr[3]= q_ros[0]
    q_tr[0:3] =  q_ros[1:]
    return q_tr

def state_model(ut, ):
    # p_robot : the position of the rigid end
    # return object position and contact force in the next timestep
    fc = [] #todo
    Rut= tr.rotation_matrix(np.linalg.norm(ut[3:]), ut[3:])[0:3,0:3]
    p_robot = p_robot_ + ut[0:3]
    R_robot = np.dot(R_robot_,Rut)

    def forward(x):
        # compute object position and vacuum contact config
        # x: change of contact config
        Rx = tr.rotation_matrix(np.linalg.norm(x[3:]), x[3:])[0:3,0:3]
        p_config = p_config_ + x[0:3]
        R_config = np.dot(R_config_, Rx)
        angle, direc, point = tr.rotation_from_matrix(np.append(np.append(R_config,[[0,0,0]],axis=0),np.array([[0,0,0,1]]).T,axis=1))
        x_config = np.append(p_config,angle*direc)
        p_contact = p_robot + np.dot(R_robot, p_config)
        R_contact = np.dot(R_robot,R_config)
        p_obj = p_contact + np.dot(R_contact, -fc)
        R_obj = R_contact
        angle, direc, point = tr.rotation_from_matrix(np.append(np.append(R_obj,[[0,0,0]],axis=0),np.array([[0,0,0,1]]).T,axis=1))
        x_obj = np.append(p_obj,angle*direc)
        return x_obj, x_config


    def potential_energy(x):
        g = 9.8
        obj_m = 0 #todo
        x_obj, x_config = forward(x)
        K_config = [] #todo
        energy = -obj_m*g*x_obj[2] + 0.5*np.dot(x_config,np.dot(K_config,x_config))
        return energy

    x0 = -ut
    res = minimize(potential_enery, x0, method='Nelder-Mead', tol=1e-4)

    x_obj, x_config = forward(res.x)
    f_contact = wrenchMapping(x_config)

    return x_obj, f_contact
