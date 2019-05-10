from module_task_model import *
import matplotlib.pyplot as plt
'''
map = compliantMapping('vc_calibration_Nmm.json')
err_torque = np.zeros(map.N_sample)
err_force = np.zeros(map.N_sample)

for i in range(map.N_sample):
    x_config = map.ref_poses[i]
    y_wrench = map.ref_wrenches[i]
    K, x0, y0 = map.local_lr_possemidef(x_config.reshape(1, -1), 'config', 'wrench')
    y_wrench_predict = np.dot(K, (x_config - x0).T) + y0

    err_force[i] = np.linalg.norm(y_wrench[0:3] - y_wrench_predict[0:3])
    err_torque[i] = np.linalg.norm(y_wrench[3:5] - y_wrench_predict[3:5])
'''
N_sample = 1697

# figure out the best pos weights to find NearestNeighbors
K_neighbors = [20,22,25,28,30]
alpha_list = [0.08,0.1,0.175,0.25,0.35,0.5]


err_trans = np.zeros([len(K_neighbors),len(alpha_list), N_sample])
err_rots = np.zeros([len(K_neighbors),len(alpha_list), N_sample])

for k in range(len(K_neighbors)):
    K_neighbor = K_neighbors[k]
    print('K_neighbor:', K_neighbor)
    for a in range(len(alpha_list)):
        alpha = alpha_list[a]
        print('alpha:', alpha)
        map = gripperMapping('vc_calibration_Nmm.json', alpha, K_neighbor)
        for i in range(map.N_sample):
            x = map.ref_wrenches[i]
            y = map.ref_poses[i]
            K,x0,y0 = map.local_lr_possemidef(x.reshape(1,-1), 'wrench', 'config')
            y_predict = np.dot(K,(x-x0).T) + y0
            print(y_predict.shape)

            err_trans[k,a,i] = np.linalg.norm(y[0:3] - y_predict[0:3])
            err_rots[k,a,i] = np.linalg.norm(y[3:5] - y_predict[3:5])

mean_f = np.mean(err_trans,axis=2)
var_f = np.var(err_trans,axis=2)
mean_t = np.mean(err_rots,axis=2)
var_t = np.var(err_rots,axis=2)

print('done')