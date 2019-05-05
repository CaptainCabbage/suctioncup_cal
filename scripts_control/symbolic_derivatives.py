from sympy import *
import numpy as np
from sympy.algebras.quaternion import Quaternion
R = sympy.MatrixSymbol('R', 3, 3)
x = sympy.MatrixSymbol('x',3,1)
y = x.T*R*x
Y =y.as_explicit()
J = Y.jacobian(x)
J_lam = sympy.lambdify((R,x),J,'numpy')
R_v = np.random.rand(3,3)
x_v = np.random.rand(3,1)
J_lam(R_v,x_v)

def J_env():
    # return the jacobian of the external contact constraints wrt to obj & robot 7D cartesian
    x_o = MatrixSymbol('x_o',7,1) # object: x y z q0 qx qy qz
    x_r = MatrixSymbol('x_r',7,1) # robot
    q_o = Quaternion(x_o[3],x_o[4],x_o[5],x_o[6])
    R_o = q_o.to_rotation_matrix()
    q_r = Quaternion(x_r[3],x_r[4],x_r[5],x_r[6])
    R_r = q_r.to_rotation_matrix()
    pass
