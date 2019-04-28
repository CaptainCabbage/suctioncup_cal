#!/usr/bin/env python

from modules import *

np.set_printoptions(precision=4, suppress=True)

task_model = taskModel('vc_calibration.json','task_model.json')
robot = abbRobot()
robot.initialize()

print('go to: '),
print(task_model.ref_act_traj[0])
raw_input()

robot.go(task_model.ref_act_traj[0])
raw_input('Please turn on vacuum')
rospy.sleep(5)

for i in range(task_model.total_timestep-1):
    robot_cartesian = robot.get_current_cartesian()
    cur_wrench = task_model.wrench_compensation(robot.current_ft(), robot_cartesian[3:])

    task_model.state_estimation(cur_wrench, robot_cartesian)
    ut = task_model.position_optimal_control(cur_wrench).reshape(-1)
    robot_ut = task_model.robot2actual(ut)
    print('ref control input:', task_model.ref_act_traj[i+1])
    print('control input: ', robot_ut,'press enter to confirm')
    raw_input()
    robot.go(task_model.ref_act_traj[i+1])
    task_model.current_timestep += 1

print("Traj execution stopped")