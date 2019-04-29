#!/usr/bin/env python

from modules import *

np.set_printoptions(precision=4, suppress=True)

task_model = taskModel('vc_calibration.json','task_model2.json')

robot = abbRobot()
robot.initialize()
robot.SetSpeed(5,3)
p0 = np.zeros(7)
p0[0:3] = task_model.ref_act_traj[0,0:3]
p0[3:] = [0,1,0,0]
print('go to: '),
print(p0)
raw_input()

robot.go(p0)
raw_input('Please turn on vacuum')

print('go to: '),
print(task_model.ref_act_traj[0])
robot.go(task_model.ref_act_traj[0])
for i in range(task_model.total_timestep-1):
    task_model.current_timestep = i

    robot_cartesian = robot.get_current_cartesian()
    cur_wrench = task_model.wrench_compensation(robot.current_ft(), robot_cartesian[3:])

    task_model.state_estimation(cur_wrench, robot_cartesian)
    ut = task_model.position_optimal_control(cur_wrench).reshape(-1)
    # bound ut
    u_pre = task_model.end_traj[i]
    du_p = ut[0:3] - u_pre[0:3]
    du_p[du_p<-10] = -10
    du_p[du_p>10] = 10
    du_q = quat_mul(ut[3:], quat_conj(u_pre[3:]))
    du_exp = quat2exp(du_q)
    print(du_exp)
    if np.linalg.norm(du_exp) > 0.08:
        du_q = exp2quat(0.08*du_exp/np.linalg.norm(du_exp))
    ut_bounded = np.zeros(7)
    ut_bounded[0:3] = u_pre[0:3] + du_p
    ut_bounded[3:] = quat_mul(du_q,u_pre[3:])
    print('ut',ut)
    print('ut_bounded',ut_bounded)
    robot_ut = task_model.robot2actual(ut_bounded)
    print('ref control input:', task_model.ref_act_traj[i+1])
    print('control input: ', robot_ut,'press enter to confirm')
    raw_input()
    robot.go(robot_ut)
    #robot.go(task_model.ref_act_traj[i+1])

print("Traj execution stopped")
