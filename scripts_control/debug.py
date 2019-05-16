
from modules import *
from module_task_model import *
import numpy as np
import time
np.set_printoptions(precision=4, suppress=True)

task_model = taskModel2('vc_calibration_Nmm.json','task_model_90.json')

f_config_ = np.array([0.402,0.0861,0.4615,4.4951,-31.201,-6.8644])
f_config_ = np.array([-0.028,-0.2789,-5.0624,-50.8237,-14.2027,-14.2948])
f_config_2 = np.array([-1,0,5,50,-500,0])
v_obj_star = np.array([1.3404, 0, 0.6702, 0, 0.0175, 0])

task_model.state_estimation(-f_config_, task_model.actual_start)
task_model.current_timestep += 1
task_model.state_estimation(-f_config_, task_model.actual_start)
dut=task_model.position_optimal_control(-f_config_, v_obj_star)
#print(dut)
'''
robot_cartesian= task_model.actual_start

dx_robot_spatial = dut[6:12]#adjointTrans(quat2rotm(robot_cartesian[3:]), robot_cartesian[0:3]).dot(dx_robot_body)
print(dx_robot_spatial)
ub_x = np.array([5,5,5,5*np.pi/180,5*np.pi/180,5*np.pi/180])
lb_x = -ub_x
dx_robot_spatial = np.minimum(dx_robot_spatial, ub_x)
dx_robot_spatial = np.maximum(dx_robot_spatial, lb_x)
print(dx_robot_spatial)
robot_ut=np.zeros(7)
robot_ut[0:3]=robot_cartesian[0:3] + dx_robot_spatial[0:3]
robot_ut[3:]= quat_mul(exp2quat(dx_robot_spatial[3:]), robot_cartesian[3:])
'''
robot_ut = dut
print('current position:',task_model.actual_start)
actual_ut = task_model.robot2actual(robot_ut)

print('control input: ', actual_ut)
