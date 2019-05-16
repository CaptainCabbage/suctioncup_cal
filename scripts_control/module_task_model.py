from modules import *
from symbolic_derivatives import *
from scipy.optimize import LinearConstraint
from cvxopt import matrix, solvers
import time

class taskModel2():

    def __init__(self, mapping_file, task_file):

        with open(task_file) as f:
            task_data = json.load(f)
        # task and obj param
        self.obj_m = task_data["object_mass"]
        self.gravity = 9.8
        self.obj_lx = task_data["object_lx"]
        self.obj_ly = task_data["object_ly"]
        self.obj_lz = task_data["object_lz"]
        # contact
        self.pc = np.squeeze(np.array(task_data["finger_contact_position"]).T)#np.array([self.obj_lx*0.8,0,self.obj_lz]); # contact position relative to the object center
        self.Rc = np.array(task_data["finger_contact_rotm"])#np.array([[1,0,0],[0,-1,0],[0,0,-1]])# contact orientation
        self.qc = rotm2quat(self.Rc)
        self.fric_mu = task_data["fric_coef"]
        self.gripper_length=task_data["gripper_length"]

        # todo:import computed obj/contact/robot end traj/actual robot traj
        self.origin = np.array(task_data["origin"])
        self.ref_obj_traj=np.array(task_data["object_trajectory"]).T
        self.ref_obj_vel_traj=np.array(task_data["object_vel_trajectory"]).T
        self.actual_start=np.array(task_data["actual_start"]).reshape(-1)

        self.total_timestep = self.ref_obj_traj.shape[0]
        self.current_timestep=0

        # to record estimated/observed states during execution
        self.obj_traj=np.zeros([self.total_timestep,7])
        self.contact_traj=np.zeros([self.total_timestep,7]) # contact force!
        self.config_traj = np.zeros([self.total_timestep,7])
        self.end_traj=np.zeros([self.total_timestep,7])
        self.act_traj=np.zeros([self.total_timestep,7])
        self.force_traj=np.zeros([self.total_timestep,6])

        # env contact in the obj frame
        self.envc1 = np.squeeze(np.array(task_data["env_contact1"]).T)#[self.obj_lx,self.obj_ly,-self.obj_lz];
        self.envc2 = np.squeeze(np.array(task_data["env_contact2"]).T)#[self.obj_lx,-self.obj_ly,-self.obj_lz];
        self.envc1_w = self.ref_obj_traj[0,0:3] + np.dot(quat2rotm(self.ref_obj_traj[0,3:]),self.envc1)
        self.envc2_w = self.ref_obj_traj[0,0:3] + np.dot(quat2rotm(self.ref_obj_traj[0,3:]),self.envc2)

        # compliant mapping
        self.vc_mapping = gripperMapping(mapping_file)
        #self.vc_mapping = compliantMapping(mapping_file)

        #
        self.J_env = func_J_env(self.envc1,self.envc1_w,self.envc2, self.envc2_w)

    def wrench_compensation(self, raw_wrench, q_robot):
        # raw_wrench -> actual wrench applied on the right end
        #R: raw_wrench corresponding robot orientation
        R = quat2rotm(q_robot)
        G = [-0.206,0.00476,-1.47]#[-0.0435, -0.0621, -1.4]
        P = [-0.00235,-0.00548,-0.0679]#[0.000786, -0.00269, -0.0709] # center of mass
        ft_offset = [0.269,2.49,-2.74,0.254,-0.0774,0.00123]#[ 1.03, 2.19, -2.28, 0.215, -0.0865, -0.0101]
        p_st =np.array([0, 0, 0.0962])
        #q_st =  [0.5556, 0, 0, 0.8315]; #qw: 0.5556 qx: 0 qy: 0 qz: 0.8315
        #R_st = np.array([[-0.3827,-0.9239,0],[0.9239,-0.3827,0],[0,0,1.0000]]) #112.5 degree
        R_st = np.array([[0.9239,-0.3827,0],[0.3827,0.9239,0],[0,0,1]]) #22.5 degree sensor to tool frame
        # sensor frame to tool frame
        Adg = adjointTrans(R_st,p_st)
        ft_t = np.dot(Adg.T,raw_wrench)

        F = np.dot(R.T,G)
        T = np.cross(P, F)
        ft_tool = ft_t + ft_offset - np.concatenate((F,T))
        ft_tool[3:] = 1000*ft_tool[3:]
        return ft_tool

    def state_estimation(self, ft_force, actual_cartesian):
        i = self.current_timestep
        print('Timestep'),
        print(i)
        print('State estimation:')
        # observation
        self.act_traj[i] = actual_cartesian
        self.end_traj[i] = self.actual2robot(actual_cartesian)
        #print('current robot:',self.end_traj[i])

        # estimation
        x_config_guess = self.vc_mapping.Mapping(-ft_force.reshape(1,-1), 'wrench', 'config').reshape(-1)
        print('x_config_guess',x_config_guess)
        #print('x_config_guess', x_config_guess)

        # use x-config guess from mapping(inaccurate) and reference obj traj (assume small error) to estimate the true state
        if i==0:

            self.obj_traj[i,:] = self.ref_obj_traj[i, :]
            self.config_traj[i] = g2cart(np.linalg.multi_dot([cart2g_inv(np.concatenate((self.pc,self.qc))),cart2g_inv(self.obj_traj[i]),cart2g(self.end_traj[i])]))

        else:
            x = self.linear_state_estimation(self.end_traj[i], self.end_traj[i-1], self.config_traj[i-1],self.obj_traj[i-1], ft_force)
            x_obj_spatial = np.dot(adjointTrans(quat2rotm(self.obj_traj[i-1,3:]),self.obj_traj[i-1,0:3]),x[0:6])
            dp_obj = x_obj_spatial[0:3]
            dq_obj = exp2quat(x_obj_spatial[3:6])
            dp_config = x[6:9]
            dq_config = exp2quat(x[9:12])
            self.obj_traj[i,0:3] = self.obj_traj[i-1,0:3] + dp_obj
            self.obj_traj[i,3:] = quat_mul(dq_obj, self.obj_traj[i-1,3:])
            self.config_traj[i, 0:3] = self.config_traj[i-1,0:3] + dp_config
            self.config_traj[i, 3:] = quat_mul(dq_config, self.config_traj[i-1,3:])

        print('estimated obj', self.obj_traj[i])

        # compute contact
        self.contact_traj[i,0:3] = np.dot(quat2rotm(self.obj_traj[i,3:]),self.pc) + self.obj_traj[i,0:3]
        self.contact_traj[i,3:] = quat_mul(self.qc,self.obj_traj[i,3:])
        print('estimated config', self.config_traj[i])

        R_config = quat2rotm(self.config_traj[i,3:])
        self.force_traj[i,:] = np.dot(adjointTrans_inv(R_config,self.config_traj[i,0:3]).T,-ft_force)# adjoint trans

    def position_optimal_control(self, ft_force, v_obj_star):
        # return
        print('OPTIMAL CONTROL')
        print('sensed force:', ft_force)
        print('obj velocity goal:', v_obj_star)

        i = self.current_timestep

        x_robot_=self.end_traj[i]
        p_config_ = self.config_traj[i,0:3]
        R_config_ = quat2rotm(self.config_traj[i,3:])
        f_config_ = -ft_force
        adg_config_T = adjointTrans_inv(R_config_,p_config_).T
        f_contact =  np.dot(adg_config_T,-ft_force)

        x_obj_ = self.obj_traj[i]

        p_robot_ = x_robot_[0:3]
        R_robot_ = quat2rotm(x_robot_[3:])
        p_obj_ = x_obj_[0:3]
        q_obj_ = x_obj_[3:]
        R_obj_ = quat2rotm(q_obj_)

        R_contact_ = np.dot(R_obj_,self.Rc)
        p_contact_ = p_obj_ + np.dot(R_obj_,self.pc)


        x_config_ = np.append(p_config_,rotm2exp(R_config_))
        K_config = self.vc_mapping.Kfx(x_config_.reshape(1, -1)) #force exert on the rigid end relative to rigid end pos change
        #K_config = -np.diag([10,10,10,200,200,200])
        K_spring = K_config
        f_spring_ = f_config_
        #print('K_spring',K_spring)

        # constraints matrix
        #1 env_constraints
        J_env = np.dot(self.J_env(x_obj_.reshape(7,1), x_robot_.reshape(7,1)), J_general_vel(x_obj_, x_robot_)) # J_env*x = 0
        # force eq
        J_co = adjointTrans_inv(self.Rc,self.pc).T
        G_o = np.concatenate((np.dot(R_obj_.T,[0,0,-self.obj_m*self.gravity]),np.zeros(3)),axis = 0)
        #print('G_o',G_o)

        ADG_env = np.zeros([6,6])
        ADG_env[0:3,0:3] = R_obj_.T
        ADG_env[0:3,3:] = R_obj_.T
        ADG_env[3:,0:3] = np.dot(skew_sym(self.envc1),R_obj_.T)
        ADG_env[3:,3:] = np.dot(skew_sym(self.envc2), R_obj_.T)
        J_force = np.concatenate((ADG_env,J_co),axis=1)


        J1 = np.zeros([12,24])
        J1[0:6,0:12] = J_env
        J1[6:,12:] = J_force

        #2 goal satisfy
        # vel goal
        # Gv*v = v_goal
        Sv = np.identity(6)
        J2 = np.concatenate((Sv,np.zeros([6,18])),axis=1)
        #constraints2 = LinearConstraint(J2, v_obj_star, v_obj_star)
        # force goal
        # Gf*fcontact = fcontact_goal

        # constraint 4: in the friction cone: only consider about the env force for now
        J4 = np.zeros([16,24])
        for k in range(8):
            d_k = np.array([-np.sin(k*np.pi/4),-np.cos(k*np.pi/4),self.fric_mu])
            J4[k,12:15] = d_k
            J4[k+8,15:18] = d_k

        # constraints on env contact force
        J5 = np.zeros([3,24])
        J5[0,14] = J5[1,12] = J5[2,13] = J5[2,16] =1
        J5[0,17] = J5[1,15] = -1

        # use quadprog
        P = np.identity(24)
        P[21:,21:] = 0.0008*np.identity(3)
        p = np.zeros(24)
        p[18:] = -f_contact
        p[21:] = 0.0008*p[21:]

        A = np.concatenate((J1[6:,:],J2,J5))
        b = np.concatenate((-G_o,v_obj_star,np.zeros(3)))

        G_bound1 = np.zeros([2,24])
        G_bound1[0,14] = G_bound1[1,17] = -1

        h_lb1 = np.ones(2)*3

        G_bound2 = np.zeros([1,24])
        G_bound2[0,20] = 1



        G = np.concatenate((-J4, G_bound1,G_bound2))
        h = np.concatenate((np.zeros(16),-h_lb1,np.ones(1)*20))

        #G = -J4
        #h = np.zeros(16)
        solvers.options['show_progress'] = False

        sol = solvers.qp(matrix(P), matrix(p), matrix(G), matrix(h), matrix(A), matrix(b))
        print(sol['status']),
        print('solution after total iterations of'),
        print(sol['iterations'])
        print(np.array(sol['x']).reshape(-1))
        res_x = np.array(sol['x']).reshape(-1)

        x_obj_spatial = np.dot(adjointTrans(R_obj_, p_obj_), res_x[0:6])
        dp_obj = x_obj_spatial[0:3]
        dq_obj = exp2quat(x_obj_spatial[3:6])
        p_obj = p_obj_ + dp_obj
        q_obj = quat_mul(dq_obj, q_obj_)

        K_inv = np.zeros([6,6])
        K_inv[0:5,0:5] = np.linalg.inv(K_spring[0:5,0:5])
        df_contact = res_x[18:] - f_contact
        df_config = np.dot(adjointTrans(R_config_, p_config_).T,df_contact)
        dx_config = np.dot(K_inv, df_config)
        ub_x = np.array([1, 1, 2, 1 * np.pi / 180, 1 * np.pi / 180, 1 * np.pi / 180])
        lb_x = -ub_x
        dx_config = np.minimum(dx_config, ub_x)
        dx_config = np.maximum(dx_config, lb_x)
        x_config = x_config_ + dx_config
        print('dx_config',dx_config)
        print('x_config',x_config)


        #x_config = self.vc_mapping.Mapping(res_x[18:].reshape(1,-1),'contact_wrench','config').reshape(-1)
        R_config = exp2rotm(x_config[3:])
        p_config = x_config[0:3]
        gr = np.linalg.multi_dot([homo_g(quat2rotm(q_obj),p_obj), homo_g(self.Rc,self.pc), homo_g(R_config,p_config)])
        x_robot = np.concatenate((gr[0:3,-1].reshape(-1),rotm2quat(gr[0:3,0:3]).reshape(-1)))

        return x_robot

    def linear_state_estimation(self, x_robot, x_robot_, x_config_, x_obj_, ft_force):
        # x: v_obj_body, v_config_body
        x_config_guess = self.vc_mapping.Mapping(-ft_force.reshape(1, -1), 'wrench', 'config').reshape(-1)
        dx_config_guess = x_config_guess - np.concatenate((x_config_[0:3],quat2exp(x_config_[3:])))
        R_robot = quat2rotm(x_robot[3:])
        R_robot_ = quat2rotm(x_robot_[3:])
        # compute robot spatial vel
        v_robot_spatial = np.concatenate((np.linalg.multi_dot([-R_robot,R_robot_.T,x_robot_[0:3]])+x_robot[0:3],rotm2exp(np.dot(R_robot,R_robot_.T))))
        print('v_robot_spatial',v_robot_spatial)
        # compute robot constraint matrix: J_robot*x = v_robot_spatial
        J_robot = np.zeros((6,12))
        J_robot[0:6,0:6] = adjointTrans(quat2rotm(x_obj_[3:]), x_obj_[0:3])
        J_robot[0:6, 6:] = adjointTrans(self.Rc, self.pc)
        # compute env constraint matrix: J_env*x = 0
        J_env = np.dot(self.J_env(x_obj_.reshape(7, 1), x_robot_.reshape(7, 1)),
                       J_general_vel(x_obj_, x_robot_))  # J_env*x = 0
        # reduce constraints matrix rows
        M = np.concatenate((np.concatenate((J_env, J_robot)), np.concatenate((np.zeros(6),v_robot_spatial)).reshape(-1,1)),axis=1)

        Q,R = np.linalg.qr(M)
        r_M = np.linalg.matrix_rank(M)
        A = R[0:r_M,0:-1]
        b = R[0:r_M,-1]
        # cost function
        P = np.identity(12)
        p = np.concatenate((np.zeros(6),-dx_config_guess))
        #G = np.concatenate((np.identity(12),-np.identity(12)))
        adg_obj =adjointTrans(quat2rotm(x_obj_[3:]), x_obj_[0:3])
        G= -np.concatenate((np.zeros(6),adg_obj[2])).reshape(1,-1)
        h = np.array([-(x_obj_[2] - self.obj_lz)])
        G_bound = np.concatenate((np.identity(12),-np.identity(12)))
        bound = np.array([3,3,3,3*np.pi/180,3*np.pi/180,3*np.pi/180,3,3,3,3*np.pi/180,3*np.pi/180,3*np.pi/180])
        #h = np.concatenate((bound,bound))
        #G = np.concatenate((G,G_bound))
        #h = np.concatenate((h,bound,bound))

        solvers.options['show_progress'] = True
        sol = solvers.qp(matrix(P), matrix(p), matrix(G), matrix(h), matrix(A), matrix(b))
        res_x = np.array(sol['x']).reshape(-1)
        print(sol['status']),
        print('solution after total iterations of'),
        print(sol['iterations'])
        print(res_x)
        return res_x

    def actual2robot(self,p_actual_):
        # convert actual robot position to execute to robot rigid end position
        p_actual = np.copy(p_actual_)
        p_rigid = np.zeros(7)
        p_rigid[3:] = p_actual[3:]
        p_rigid[0:3] = p_actual[0:3] + np.dot(quat2rotm(p_actual[3:]),[0,0,self.gripper_length]) - self.origin.reshape(-1)
        return p_rigid

    def robot2actual(self, p_rigid_):
        p_rigid = np.copy(p_rigid_)
        p_actual = np.zeros(7)
        p_actual[3:]=p_rigid[3:]
        p_actual[0:3] = p_rigid[0:3] - np.dot(quat2rotm(p_rigid[3:]),[0,0,self.gripper_length]) + self.origin.reshape(-1)
        return p_actual

class gripperMapping():

    def __init__(self, mapping_file):

        K_neighbor_wrench = 20
        K_neighbor_pos = 60
        alpha_wrench = 0.25
        self.alpha_wrench =[1,1,1,alpha_wrench,alpha_wrench,alpha_wrench]
        self.alpha_pos = [1,1,1,50,50,50]

        with open(mapping_file) as f:
            json_data = json.load(f)
        self.ref_cartesians = np.array(json_data["ref_cartesians"]).T
        self.ref_wrenches = np.array(json_data["ref_wrenches"]).T
        self.ref_contact_wrenches = np.array(json_data["ref_contact_wrenches"]).T
        #print(self.ref_cartesians)
        self.ref_poses = toExpmap(self.ref_cartesians)

        self.contact_wrench_nbrs = NearestNeighbors(n_neighbors=K_neighbor_wrench, algorithm='ball_tree').fit(\
        self.ref_contact_wrenches*self.alpha_wrench)

        self.wrench_nbrs = NearestNeighbors(n_neighbors=K_neighbor_wrench, algorithm='ball_tree').fit(\
        self.ref_wrenches*self.alpha_wrench)

        self.pos_nbrs = NearestNeighbors(n_neighbors=K_neighbor_pos, algorithm='ball_tree').fit(\
        self.ref_poses*self.alpha_pos)

        self.N_sample = self.ref_cartesians.shape[0]

    def __get_param(self, X_name, Y_name):
        if X_name == 'wrench':
            _Xref_nbrs = self.wrench_nbrs
            _Xref = self.ref_wrenches
            _alpha = self.alpha_wrench
        elif X_name == 'contact_wrench':
            _Xref_nbrs = self.contact_wrench_nbrs
            _Xref = self.ref_contact_wrenches
            _alpha = self.alpha_wrench
        elif X_name == 'config':
            _Xref_nbrs = self.pos_nbrs
            _Xref = self.ref_poses
            _alpha = self.alpha_pos
        else:
            raise NameError('the 1st param should only be wrench, contact_wrench or config')

        if Y_name == 'wrench':
            _Yref = self.ref_wrenches
        elif Y_name == 'contact_wrench':
            _Yref = self.ref_contact_wrenches
        elif Y_name == 'config':
            _Yref = self.ref_poses
        else:
            raise NameError('the 2nd param should only be wrench, contact_wrench or config')

        return _Xref, _Xref_nbrs, _Yref, _alpha

    def local_lr_possemidef(self, X, X_name, Y_name):
        # return: Y.T = K*(X-x0).T + y0.T
        #print(X)
        Xref, Xref_nbrs, Yref, alpha = self.__get_param(X_name,Y_name)
        X_ = X*alpha
        distances, indices = Xref_nbrs.kneighbors(X_)
        #itself = np.argwhere(distances==0)
        #if len(itself)!=0:
        #    indices = np.delete(indices,np.argwhere(distances==0)[0])
        X_ind = np.squeeze(Xref[indices])
        Y_ind = np.squeeze(Yref[indices])
        #print(X_ind)
        #print(Y_ind)

        x0 = np.mean(X_ind, axis=0)
        y0 = np.mean(Y_ind, axis=0)
        r_max = max(np.linalg.norm(X_ind[:, :-1] - x0[:-1],axis=1))
        K_T = posdef_estimation(X_ind[:, :-1] - x0[:-1],Y_ind[:,:-1]- y0[:-1])
        K = np.zeros([6,6])
        K[0:5,0:5] = K_T.T
        return K,x0,y0,r_max

    def Kfx(self,x_pos):
        K, x0,y0,r = self.local_lr_possemidef(x_pos, 'config', 'wrench')
        return K
        #return self.local_lr(x_pos,'config','wrench').coef_

    def Mapping(self, x, X_name, Y_name):

        K, x0, y0,r_max = self.local_lr_possemidef(x, X_name, Y_name)
        x = x.reshape(-1)
        r = np.linalg.norm(x[:-1]-x0[:-1])
        if r > 1.2*r_max:
            x = x*1.2*r_max/r
        y_predict = np.dot(K, (x - x0).T).reshape(-1) + y0

        return y_predict
