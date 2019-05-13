#!/usr/bin/env python

from modules import *
from module_task_model import *

np.set_printoptions(precision=4, suppress=True)
np.random.seed(100)

task_model = taskModel2('vc_calibration_Nmm.json','task_model_new.json')

robot = abbRobot()
robot.initialize()
robot.SetSpeed(5,3)
'''
p0 = np.zeros(7)
p0[0:3] = task_model.actual_start[0:3]
p0[2] = 520
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
#rospy.sleep(5)
'''
i=0
while i < task_model.total_timestep-1:
    print('timestep:',i,'angle', i*30/45)
    task_model.current_timestep = i

    actual_cartesian = np.array(robot.get_current_cartesian())
    robot_cartesian = task_model.actual2robot(actual_cartesian)
    cur_wrench = task_model.wrench_compensation(robot.current_ft(), actual_cartesian[3:])
    #cur_wrench = np.array([0.402,0.0861,0.4615,4.4951,-31.201,-6.8644])
    print('cur wrench',cur_wrench)

    task_model.state_estimation(cur_wrench, actual_cartesian)
    dut = task_model.position_optimal_control(cur_wrench, task_model.ref_obj_vel_traj[i]).reshape(-1)
    #dx_robot_body = dut[6:12]
    #dx_robot_spatial = adjointTrans(quat2rotm(robot_cartesian[3:]),robot_cartesian[0:3]).dot(dx_robot_body)
    dx_robot_spatial = dut[6:12]
    ub_x = np.array([5, 5, 5, 5 * np.pi / 180, 5 * np.pi / 180, 5 * np.pi / 180])
    lb_x = -ub_x
    dx_robot_spatial = np.minimum(dx_robot_spatial, ub_x)
    dx_robot_spatial = np.maximum(dx_robot_spatial, lb_x)
    robot_ut = np.zeros(7)
    robot_ut[0:3] = robot_cartesian[0:3] + dx_robot_spatial[0:3]
    robot_ut[3:]=quat_mul(exp2quat(dx_robot_spatial[3:]), robot_cartesian[3:])
    print(robot_cartesian)
    print('dx_robot_spatial:',dx_robot_spatial)

    actual_ut = task_model.robot2actual(robot_ut)

    print('control input: ', actual_ut,'press enter to confirm, input 0 to reject')
    var = raw_input()
    if var == '0':
        i+=1
        continue
    robot.go(actual_ut)
    rospy.sleep(2)
    #robot.go(task_model.ref_act_traj[i+1])
    i+=1

print("Traj execution stopped")
