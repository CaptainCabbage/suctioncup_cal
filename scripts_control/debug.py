
from modules import *
from module_task_model import *
import numpy as np
import time
np.set_printoptions(precision=4, suppress=True)

task_model = taskModel2('vc_calibration_Nmm.json','task_model_90.json')

robot_cartesian = [110.7,400.5,469.0,0.0, 0.9954, 0.0, -0.0954]
f_config_ = np.array([-0.7226,  0.0918, -3.684 , 16.1009, 13.5253, -4.4922])
v_obj_star = np.array([0.5027,0,0.5027,0,0.0131,0])

task_model.state_estimation(-f_config_, task_model.actual_start)
dut=task_model.position_optimal_control(-f_config_, v_obj_star)
task_model.current_timestep += 1
task_model.state_estimation(-f_config_, robot_cartesian)
'''
#dut=task_model.position_optimal_control(-f_config_, v_obj_star)
#print(dut)

robot_ut = dut
print('current position:',task_model.actual_start)
actual_ut = task_model.robot2actual(robot_ut)

print('control input: ', actual_ut)
'''