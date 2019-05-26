#!/usr/bin/env python

from modules import *
from module_task_model import *

np.set_printoptions(precision=4, suppress=True)
np.random.seed(100)

task_model = taskModel2('vc_calibration_Nmm.json','./task_files/task_openloop.json')

task_file = './task_files/task_openloop.json'
with open(task_file) as f:
    task_data = json.load(f)
ref_end_traj=np.array(task_data["openloop_robot_end_trajectory"]).T

error = -2
task_model.origin[2] = task_model.origin[2] + error

bagname = 'openloop_' + str(error) +'mm_'+ str(int(time.time())) + '.bag'
robot = abbRobot(bagname)
robot.initialize()
robot.SetSpeed(20,20)

p0 = np.array([72,400.2,495,0,1,0,0])
p0[0:2] = task_model.actual_start[0:2]
raw_input('go to: '+ str(p0))
robot.go(p0)

raw_input('go to: ' + str(task_model.actual_start))
robot.go(task_model.actual_start)
raw_input('press enter to start')

for i in range(ref_end_traj.shape[0]):
    pi = ref_end_traj[i]
    pi[2] = pi[2] + error
    pi_actual = task_model.robot2actual((pi))
    #raw_input("Please check the robot position:" + str(pi_actual))
    robot.go(pi_actual)

    while np.linalg.norm(np.subtract(robot.get_current_cartesian(),pi_actual)[0:3]) > 0.1:
        continue

    #rospy.sleep(2)

    # record ft data
    actual_cartesian = np.array(robot.get_current_cartesian())
    robot_cartesian = task_model.actual2robot(actual_cartesian)
    cur_wrench = task_model.wrench_compensation(robot.current_ft(), actual_cartesian[3:])
    robot.record_ft(i,cur_wrench)

robot.stop()
print("Traj execution stopped")


