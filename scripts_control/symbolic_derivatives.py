from sympy import *
import numpy as np
from utils import *
from sympy.algebras.quaternion import Quaternion


def func_J_env(envc1_,envc1_w_,envc2_,envc2_w_):
    # return the function of jacobian of the external contact constraints wrt to obj & robot 7D cartesian
    envc1 = Matrix(envc1_.tolist())
    envc1_w = Matrix(envc1_w_.tolist())
    envc2 = Matrix(envc2_.tolist())
    envc2_w = Matrix(envc2_w_.tolist())
    x_o = MatrixSymbol('x_o',7,1) # object: x y z q0 qx qy qz
    x_r = MatrixSymbol('x_r',7,1) # robot
    q_o = Quaternion(x_o[3],x_o[4],x_o[5],x_o[6])
    R_o = q_o.to_rotation_matrix()
    p_o = Matrix([x_o[0],x_o[1],x_o[2]])
    #q_r = Quaternion(x_r[3],x_r[4],x_r[5],x_r[6])
    #R_r = q_r.to_rotation_matrix()
    eqn1 = R_o*envc1 + p_o - envc1_w
    eqn2 = R_o*envc2 + p_o - envc2_w
    eqn = Matrix([eqn1,eqn2])
    J_eqn = eqn.jacobian([x_o,x_r])
    func_J =lambdify((x_o,x_r),J_eqn,'numpy')

    return func_J

def J_general_vel(x_o, x_r):
    # q_dot = J_general_vel*v
    J = np.zeros([14,12])
    J[0:3,0:3] = quat2rotm(x_o[3:])
    J[3:7,3:6] = J_w2quat(x_o[3:])
    J[7:10,6:9] = quat2rotm(x_r[3:])
    J[10:14,9:12] = J_w2quat(x_r[3:])
    return J

def J_w2quat(q):
    J = 0.5*np.array([[-q[1],-q[2],-q[3]],[q[0],-q[3],q[2]],[q[3],q[0],-q[1]],[-q[2],q[1],q[0]]])
    return J
