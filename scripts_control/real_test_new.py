#!/usr/bin/env python

from modules import *
from module_task_model import *

np.set_printoptions(precision=4, suppress=True)
np.random.seed(100)

task_model = taskModel2('vc_calibration_Nmm.json','task_model_90.json')

robot = abbRobot()
robot.initialize()
#robot.SetSpeed(5,3)
robot.SetSpeed(10,5)

p0 = np.zeros(7)
p0[0:3] = task_model.actual_start[0:3]
p0[2] = 495
p0[3:] = [0,1,0,0]
#print('go to: '),
#print(p0)
#raw_input()

#robot.go(p0)
#raw_input('Please turn on vacuum')

print('go to: '),
print(task_model.actual_start)
raw_input()
robot.go(task_model.actual_start)
raw_input('press enter to start')

i=0
while i < 120-1:
    print('timestep:',i,'angle', i*90/120)
    task_model.current_timestep = i

    actual_cartesian = np.array(robot.get_current_cartesian())
    robot_cartesian = task_model.actual2robot(actual_cartesian)
    cur_wrench = task_model.wrench_compensation(robot.current_ft(), actual_cartesian[3:])
    cur_wrench[3:] = cur_wrench[3:] + np.array([-12,11,5.4])

    print('cur wrench',cur_wrench)

    task_model.state_estimation(cur_wrench, actual_cartesian)
    q_obj_w = quat_mul(task_model.ref_obj_traj[i+1,3:],quat_conj(task_model.obj_traj[i,3:]))
    v_obj_w = np.concatenate((task_model.ref_obj_traj[i+1,0:3]-task_model.obj_traj[i,0:3],quat2exp(q_obj_w)))
    v_obj_b = adjointTrans(quat2rotm(task_model.obj_traj[i,3:]).T, -np.dot(quat2rotm(task_model.obj_traj[i,3:]).T,task_model.obj_traj[i,0:3])).dot(v_obj_w)
    print('v_obj_b',v_obj_b)
    robot_ut = task_model.position_optimal_control(cur_wrench, task_model.ref_obj_vel_traj[i])

    actual_ut = task_model.robot2actual(robot_ut)

    print('control input: ', actual_ut,'press enter to confirm, input 0 to reject')
    var = raw_input()
    #if var == '0':
    #    i+=1
    #    continue
    robot.go(actual_ut)
    rospy.sleep(1)
    #robot.go(task_model.ref_act_traj[i+1])
    i+=1

print("Traj execution stopped")
