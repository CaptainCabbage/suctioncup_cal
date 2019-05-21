import numpy as np
import tf.transformations as tr
import scipy

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

def homo_g(R,p):
    g= np.identity(4)
    g[0:3,0:3] = R
    g[0:3,-1] = p
    return g

def cart2g(p):
    g= np.identity(4)
    g[0:3,0:3] = quat2rotm(p[3:])
    g[0:3,-1] = p[0:3]
    return g

def cart2g_inv(p):
    g= np.identity(4)
    g[0:3,0:3] = quat2rotm(p[3:]).T
    g[0:3,-1] = -np.dot(g[0:3,0:3],p[0:3])
    return g

def g2cart(g):
    p = np.zeros(7)
    p[0:3] = g[0:3,-1]
    p[3:] = rotm2quat(g[0:3,0:3])
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

# below quat2rotm and rotm2quat all use quaternion represenation in ros (w x y z), aviod conflict with tr (x y z w)
def quat2rotm(q):
    R= tr.quaternion_matrix(quat_ros2tr(q)) # q: w x y z
    return R[0:3,0:3]

def rotm2quat(R):
    R_ = np.identity(4)
    R_[0:3,0:3] = R
    return quat_tr2ros(tr.quaternion_from_matrix(R_))

def toExpmap(p):
    # cartesian representation N x7 (px py pz w x y z) -> N x 6
    if len(p.shape) ==1:
        pos = np.zeros(6)
        pos[0:3] = p[0:3]
        angle = 2*np.arccos(p[3])
        if angle == 0:
            pos[3:] = np.zeros(3)
        else:
            scale = angle/np.sqrt(1-p[3]**2)
            pos[3:] = p[4:]*scale
    else:
        pos = np.zeros([p.shape[0],6])
        pos[:,0:3] =p[:,0:3]
        angle = 2*np.arccos(p[:,3])
        scale = np.zeros(angle.shape)
        scale[angle!=0] = angle[angle!=0]/np.sqrt(1-p[angle!=0,3]**2)
        #ref_poses[:,3:] = np.multiply(self.ref_cartesians[:,4:],np.matlib.repmat(scale,3,1).T)
        pos[:,3:] = (p[:,4:].T*scale).T

    return pos

def toCartesian(pos):
    # N x 6 -> cartesian representation N x7 (px py pz w x y z)
    if len(pos.shape) == 1:
        p=np.zeros(7)
        q = tr.quaternion_about_axis(np.linalg.norm(pos[3:]),pos[3:])
        p[0:3] = pos[0:3]
        p[3] = q[3]
        p[4:]= q[0:3]
    else:
        p=np.zeros([pos.shape[0],7])
        for i in range(pos.shape[0]):
            q = tr.quaternion_about_axis(np.linalg.norm(pos[i,3:]),pos[i,3:])
            p[i,0:3] = pos[i,0:3]
            p[i,3:] = quat_tr2ros(q)
    return p

def adjointTrans(R,p):
    P = np.array([[0,-p[2],p[1]],[p[2], 0, -p[0]],[-p[1], p[0], 0]])
    adg = np.zeros([6,6])
    adg[0:3,0:3]=R
    adg[3:,3:]=R
    adg[0:3,3:] = np.dot(P,R)
    #adg = [R,P*R;zeros(3,3) R];
    return adg

def adjointTrans_inv(R,p):
    P = np.array([[0,-p[2],p[1]],[p[2], 0, -p[0]],[-p[1], p[0], 0]])
    adg = np.zeros([6,6])
    adg[0:3,0:3]=R.T
    adg[3:,3:]=R.T
    adg[0:3,3:] = -np.dot(R.T,P)
    #adg = [R,P*R;zeros(3,3) R];
    return adg

def exp2rotm(x):
    if sum(x == 0) == 3:
        Rx = np.identity(3)
    else:
        Rx = tr.rotation_matrix(np.linalg.norm(x), x)[0:3, 0:3]
    return Rx

def rotm2exp(R):
    #R 3x3
    R_ = np.identity(4)
    R_[0:3,0:3] = R
    angle, direc, point = tr.rotation_from_matrix(R_)
    x = angle*direc
    return x

def quat2exp(q):
    q = q/np.linalg.norm(q)
    if q[0]**2 == 1:
        w = np.zeros(3)
    else:
        w = 2*np.arccos(q[0])*q[1:]/((1-q[0]**2)**0.5)
    return w

def exp2quat(x):
    p=np.zeros(4)
    q = tr.quaternion_about_axis(np.linalg.norm(x),x)
    p[0] = q[3]
    p[1:]= q[0:3]
    return p

def quat_conj(q):
    q_conj = np.zeros(4)
    q_conj[0] = q[0]
    q_conj[1:] = -q[1:]
    return q_conj

def quat_mul(q1,q2):
    #return q1 o q2
    q=np.zeros(4)
    q[0] = q1[0]*q2[0] - np.dot(q1[1:],q2[1:])
    q[1:] = q1[0]*q2[1:] + q2[0]*q1[1:] + np.cross(q1[1:],q2[1:])
    q = q/np.linalg.norm(q)

    return q

def quat_mul_derivative_left(q0):
    der_left = np.array([[ q0[0],  q0[1],  q0[2],  q0[3]],\
    [ q0[1],  q0[0], -q0[3],  q0[2]],\
    [ q0[2],  q0[3],  q0[0], -q0[1]],\
    [ q0[3], -q0[2],  q0[1],  q0[0]]])
    return der_left

def quat_mul_derivative_right(q0):
    der_right = np.array([[ q0[0],  q0[1],  q0[2],  q0[3]],\
    [ q0[1],  q0[0],  q0[3], -q0[2]],\
    [ q0[2], -q0[3],  q0[0],  q0[1]],\
    [ q0[3],  q0[2], -q0[1],  q0[0]]])
    return der_right

def posdef_estimation(A,B):
    # AX=B
    # X: positive semi definete matrix
    P = np.dot(A.T,A)
    Q = np.dot(B.T,B)
    sigma_P_sq,U_P = np.linalg.eig(P)
    sigma_P_sq[sigma_P_sq<=1e-10] = 1e-10
    #sigma_P_sq=np.real(sigma_P_sq)
    #U_P = np.real(U_P)
    Sigma_P = np.diag(np.sqrt(sigma_P_sq))
    Qt = np.linalg.multi_dot([Sigma_P, U_P.T, Q, U_P, Sigma_P])
    sigma_Qt_sq, U_Qt = np.linalg.eig(Qt)
    sigma_Qt_sq[sigma_Qt_sq<=1e-10] = 1e-10
    #sigma_Qt_sq=np.real(sigma_Qt_sq)
    #U_Qt = np.real(U_Qt)
    Sigma_Qt = np.diag(np.sqrt(sigma_Qt_sq))
    X = np.linalg.multi_dot([U_P, np.diag(1/np.sqrt(sigma_P_sq)), U_Qt, Sigma_Qt, U_Qt.T,np.diag(1/np.sqrt(sigma_P_sq)),U_P.T])

    return X

def skew_sym(x):
    X = np.array([[0,-x[2],x[1]],[x[2],0,-x[0]],[-x[1],x[0],0]])
    return X

def twist2g(t):
    theta = np.linalg.norm(t[3:])
    w = t[3:]/theta
    v = t[0:3]/theta

    g = np.identity(4)
    if np.linalg.norm(w) == 0:
        g[0:3,3] = v*theta
    else:
        g[0:3,0:3] = exp2rotm(w*theta)
        g[0:3,3] = np.dot(np.identity(3) - g[0:3,0:3],np.cross(w,v)) + np.dot(w.reshape(-1,1), w.reshape(1,-1)).dot(v)*theta

    return g

def g2twist(g):
    w = rotm2exp(g[0:3,0:3]).reshape(-1)
    theta = np.linalg.norm(w)
    if theta == 0:
        v = g[0:3,3].reshape(-1)
        t = np.concatenate((v,w))
    else:
        w = w/theta
        A = np.dot(np.identity(3) - g[0:3,0:3], skew_sym(w)) + np.dot(w.reshape(-1,1), w.reshape(1,-1))*theta
        v = np.dot(np.linalg.inv(A),g[0:3,3]).reshape(-1)
        t = np.concatenate((v,w))*theta
    return t
