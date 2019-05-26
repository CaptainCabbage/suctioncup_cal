
from modules import *
from module_task_model import *
import numpy as np
import time
np.set_printoptions(precision=4, suppress=True)

task_model = taskModel2('vc_calibration_Nmm.json','task_model_90.json')

robot_cartesian = [110.7,400.5,469.0,0.0, 0.9954, 0.0, -0.0954]
f_config_ = np.array([-0.7226,  0.0918, -3.684 , 16.1009, 13.5253, -4.4922])
v_obj_star = np.array([0.5027,0,0.5027,0,0.0131,0])

'''
task_model.state_estimation(-f_config_, task_model.actual_start)
g_goal = cart2g(task_model.ref_obj_traj[2])
g_inv = cart2g_inv(task_model.obj_traj[0])
g_b = np.dot(g_inv, g_goal)
v_obj_b = g2twist(g_b)
print('!!!v_obj_b', v_obj_b)
dut=task_model.position_optimal_control(-f_config_, v_obj_star)
task_model.current_timestep += 1
#task_model.state_estimation(-f_config_, robot_cartesian)
'''


#robot_ut = dut
#print('current position:',task_model.actual_start)
#actual_ut = task_model.robot2actual(robot_ut)

task_model.state_estimation(np.array([ 0.2001,  0.1078, -0.9709, -0.7652,  1.5876, -0.4652]),np.array([72.26, 400.22, 472.,0,1,0,0]))

task_model.current_timestep+=1

task_model.state_estimation(np.array([ 0.0349, 1.0198,-10.1373, 13.248 , 6.6394,-0.7422]), np.array([ 74.1265, 400.4944, 470.7957,-0,1,0,-0.0056]))
'''
task_model.current_timestep+=1
task_model.state_estimation(np.array([-1.003 ,  0.1049, -4.5877,  1.5108, 14.7147, -1.2643]),np.array([ 77.95  , 400.2991, 471.027 ,  -0. , 0.9999,  -0.,-0.0156]))
task_model.current_timestep+=1
task_model.state_estimation(np.array([-1.2757,  0.2377, -6.9784,  3.9926, 19.7487, -1.3256]),np.array([ 80.2304, 400.3329, 470.6574,  -0.    ,   0.9998,  -0.,-0.0222]))
'''
dut=task_model.position_optimal_control(np.array([ 0.0349, 1.0198,-10.1373, 13.248 , 6.6394,-0.7422]), v_obj_star)
print(task_model.robot2actual(dut))
