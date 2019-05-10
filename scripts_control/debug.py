
from modules import *
from module_task_model import *
import numpy as np
import time
np.set_printoptions(precision=4, suppress=True)

task_model = taskModel2('vc_calibration_Nmm.json','task_model_new.json')

f_config_ = np.array([0,0,10,0,0,0])
f_config_2 = np.array([-1,0,5,50,-500,0])
v_obj_star = np.array([1.3404, 0, 0.6702, 0, 0.0175, 0])

task_model.state_estimation(f_config_, task_model.actual_start)
task_model.current_timestep += 1
task_model.state_estimation(f_config_2, task_model.actual_start)
x=task_model.position_optimal_control(-f_config_, v_obj_star)
