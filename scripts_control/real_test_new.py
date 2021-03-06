#!/usr/bin/env python

from modules import *
from module_task_model import *

np.set_printoptions(precision=4, suppress=True)
np.random.seed(100)

task_model = taskModel2('vc_calibration_Nmm.json','task_model_90.json')

robot = abbRobot()
#robot = abbRobot('standard_' + str(int(time.time())) + '.bag')
robot.initialize()

robot.SetSpeed(20,20)

'''
p0 = np.array([72,400.2,495,0,1,0,0])
p0[0:2] = task_model.actual_start[0:2]
raw_input('go to: '+ str(p0))
robot.go(p0)
'''

print('go to: ' + str(task_model.actual_start))
robot.go(task_model.actual_start)
raw_input('press enter to start')

for i in range(120):
    print('Timestep:' + str(i) + ' angle:' + str(i*90.0/120.0))
    task_model.current_timestep = i

    actual_cartesian = np.array(robot.get_current_cartesian())
    robot_cartesian = task_model.actual2robot(actual_cartesian)
    cur_wrench = task_model.wrench_compensation(robot.current_ft(), actual_cartesian[3:])
    cur_wrench[3:] = cur_wrench[3:] + np.array([-10,12,5.4])

    print('cur wrench: ' + str(cur_wrench))

    task_model.state_estimation(cur_wrench, actual_cartesian)
    g_goal = cart2g(task_model.ref_obj_traj[i+1])
    g_inv = cart2g_inv(task_model.obj_traj[i])
    g_b = np.dot(g_inv,g_goal)
    v_obj_b = g2twist(g_b)
    print('desired v_obj_b:' + str(v_obj_b))
    #robot_ut = task_model.position_optimal_control(cur_wrench, task_model.ref_obj_vel_traj[i])
    robot_ut = task_model.position_optimal_control(cur_wrench, v_obj_b)

    if len(robot_ut) == 0:
        print('Non-Optimal Solution, BREAK')
        break

    actual_ut = task_model.robot2actual(robot_ut)

    print('control input: ' + str(actual_ut) + 'press enter to confirm, input 0 to reject')
    if i >= 0:
        var = raw_input()
        #if var == '0':
        #    i+=1
        #    continue
    robot.go(actual_ut)
    while np.linalg.norm(np.subtract(robot.get_current_cartesian(), actual_ut)[0:3]) > 0.1:
        continue
    #robot.go(task_model.ref_act_traj[i+1])

robot.stop()
print("Traj execution stopped")
