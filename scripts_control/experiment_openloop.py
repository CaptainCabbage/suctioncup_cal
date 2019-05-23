from modules import *
from module_task_model import *

np.set_printoptions(precision=4, suppress=True)
np.random.seed(100)

task_file = 'task_openloop.json'
with open(task_file) as f:
    task_data = json.load(f)
ref_actual_traj=np.array(task_data["actual_trajectory"]).T

error = 0

bagname = 'openloop_' + str(error) +'mm_'+ str(int(time.time())) + '.bag'
robot = abbRobot(bagname)
robot.initialize()
robot.SetSpeed(10,10)

p0 = np.array([77,400.2,495,0,1,0,0])
p0[0:2] = ref_actual_traj[0,0:2]
raw_input('go to: '+ str(p0))
robot.go(p0)

raw_input('go to: ' + str(ref_actual_traj[0]))
robot.go(ref_actual_traj[0])
raw_input('press enter to start')

for i in range(ref_actual_traj.shape[0]):
    pi = ref_actual_traj[i]
    pi[2] = pi[2] + error
    raw_input("Please check the robot position:" + str(pi))
    robot.go(pi)
    rospy.sleep(0.5)
    robot.record_ft(i)

print("Traj execution stopped")


