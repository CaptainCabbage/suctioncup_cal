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

        #
        self.J_env = func_J_env(self.envc1,self.envc1_w,self.envc2, self.envc2_w)

    def wrench_compensation(self, raw_wrench, q_robot):
        # raw_wrench -> actual wrench applied on the right end
        #R: raw_wrench corresponding robot orientation
        R = quat2rotm(q_robot)
        G = [-0.0435, -0.0621, -1.4]
        P =  [0.000786, -0.00269, -0.0709]
        ft_offset = [ 1.03, 2.19, -2.28, 0.215, -0.0865, -0.0101]
        p_st =np.array([0, 0, 0.0962])
        #q_st =  [0.5556, 0, 0, 0.8315]; #qw: 0.5556 qx: 0 qy: 0 qz: 0.8315
        R_st = np.array([[-0.3827,-0.9239,0],[0.9239,-0.3827,0],[0,0,1.0000]])
        # sensor frame to tool frame
        Adg = adjointTrans(R_st,p_st)
        ft_t = np.dot(Adg.T,raw_wrench)

        F = np.dot(R.T,G)
        T = np.cross(P, F)
        ft_tool = ft_t + ft_offset - np.concatenate((F,T))
        ft_tool[3:] = 1000*ft_tool[3:]
        return ft_tool

    def state_estimation(self, ft_force, robot_cartesian):
        i = self.current_timestep
        print('Timestep'),
        print(i)
        print('State estimation:')
        # observation
        self.act_traj[i] = robot_cartesian
        self.end_traj[i] = self.actual2robot(robot_cartesian)
        #print('current robot:',self.end_traj[i])
        #ft_force = self.wrench_compensation(robot_ft,robot_cartesian[3:]) TODO change this back!!

        # estimation
        x_config_guess = self.vc_mapping.Mapping(-ft_force.reshape(1,-1), 'wrench', 'config').reshape(-1)
        print('x_config_guess',x_config_guess)
        #print('x_config_guess', x_config_guess)

        # use x-config guess from mapping(inaccurate) and reference obj traj (assume small error) to estimate the true state
        if i==0:
            x = self.state_model(np.zeros(6), self.end_traj[i], x_config_guess, ft_force, self.ref_obj_traj[i])
            dp_obj = x[0:3]
            dq_obj = x[3:7] / np.linalg.norm(x[3:7])
            dp_config = x[7:10]
            dq_config = x[10:14]
            self.obj_traj[i, 0:3] = self.ref_obj_traj[i, 0:3] + dp_obj
            self.obj_traj[i, 3:] = quat_mul(dq_obj, self.ref_obj_traj[i, 3:])
            self.config_traj[i, 0:3] = x_config_guess[0:3] + dp_config
            self.config_traj[i, 3:] = quat_mul(dq_config, exp2quat(x_config_guess[3:]))

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

        # compute config
        self.contact_traj[i,0:3] = np.dot(quat2rotm(self.obj_traj[i,3:]),self.pc) + self.obj_traj[i,0:3]
        self.contact_traj[i,3:] = quat_mul(self.qc,self.obj_traj[i,3:])

        R_config = quat2rotm(self.config_traj[i,3:])
        self.force_traj[i,:] = np.dot(adjointTrans(R_config.T, -np.dot(R_config.T,self.config_traj[i,0:3]).T),-ft_force)# adjoint trans

    def position_optimal_control(self, ft_force, v_obj_star):
        # return
        print('OPTIMAL CONTROL')

        i = self.current_timestep

        x_robot_=self.end_traj[i]
        p_config_ = self.config_traj[i,0:3]
        R_config_ = quat2rotm(self.config_traj[i,3:])
        f_config_ = -ft_force

        x_obj_ = self.obj_traj[i]

        p_robot_ = x_robot_[0:3]
        R_robot_ = quat2rotm(x_robot_[3:])
        p_obj_ = x_obj_[0:3]
        q_obj_ = x_obj_[3:]
        R_obj_ = quat2rotm(q_obj_)


        x_config_ = np.append(p_config_,rotm2exp(R_config_))
        K_config = self.vc_mapping.Kfx(x_config_.reshape(1, -1)) #force exert on the rigid end relative to rigid end pos change
        #K_config = -np.diag([10,10,10,200,200,200])
        K_spring = 2*K_config
        f_spring_ = -f_config_

        # constraints matrix
        #1 env_constraints
        J_env = np.dot(self.J_env(x_obj_.reshape(7,1), x_robot_.reshape(7,1)), J_general_vel(x_obj_, x_robot_)) # J_env*x = 0
        # force eq
        J_co = adjointTrans(self.Rc.T, -np.dot(self.Rc.T,self.pc)).T
        G_o = np.concatenate((np.dot(R_obj_.T,[0,0,-self.obj_m*self.gravity]),np.zeros(3)),axis = 0)

        ADG_env = np.zeros([6,6])
        ADG_env[0:3,0:3] = R_obj_.T
        ADG_env[0:3,3:] = R_obj_.T
        ADG_env[3:,0:3] = np.dot(skew_sym(self.envc1),R_obj_.T)
        ADG_env[3:,3:] = np.dot(skew_sym(self.envc2), R_obj_.T)
        J_force = np.concatenate((ADG_env,J_co),axis=1)

        J1 = np.zeros([12,24])
        J1[0:6,0:12] = J_env
        J1[6:,12:] = J_force
        #constraints1 = LinearConstraint(J1, np.concatenate((np.zeros(6),-G_o)), np.concatenate((np.zeros(6),-G_o)))
        constraints1 = LinearConstraint(J1[6:],  -G_o,  -G_o)

        #2 goal satisfy
        # vel goal
        # Gv*v = v_goal
        Sv = np.identity(6)
        J2 = np.concatenate((Sv,np.zeros([6,18])),axis=1)
        constraints2 = LinearConstraint(J2, v_obj_star, v_obj_star)
        # force goal
        # Gf*fcontact = fcontact_goal

        #3 fcontact relationship
        # in contact frame: f_contact = adj_config(f_spring + K_spring*V_contact_at_robot)
        adg_contact = np.identity(6)
        adg_contact[0:3,3:] = skew_sym(self.pc)
        adg_robot2obj = adjointTrans(np.dot(R_robot_.T,R_obj_), np.dot(R_robot_.T,p_robot_ - p_obj_))
        adg_contact2robot = np.dot(adg_robot2obj,adg_contact)
        adg_config = adjointTrans(R_config_.T, -np.dot(R_config_.T,p_config_))
        adg_vobj = np.linalg.multi_dot([adg_config,K_spring,adg_contact2robot])
        J3 = np.zeros([6,24])
        J3[:,0:6] = -adg_vobj
        J3[:,6:12] = np.linalg.multi_dot([adg_config,K_spring,np.identity(6)])
        J3[:,18:24] = np.identity(6)
        constraints3 = LinearConstraint(J3,np.dot(adg_config,f_spring_),np.dot(adg_config,f_spring_))

        # constraint 4: in the friction cone: only consider about the env force for now
        J4 = np.zeros([16,24])
        for k in range(8):
            d_k = np.array([-np.sin(k*np.pi/4),-np.cos(k*np.pi/4),self.fric_mu])
            J4[k,12:15] = d_k
            J4[k+8,15:18] = d_k

        constraints4 = LinearConstraint(J4,np.full(16, 1), np.full(16, 1))#, keep_feasible=True)

        J_all = np.concatenate((J1,J2,J3,J4))
        cons_lb = np.concatenate((np.concatenate((np.zeros(6),-G_o)),v_obj_star,np.dot(adg_config,f_spring_),
                                 np.full(16, 0)))
        cons_ub = np.concatenate((np.concatenate((np.zeros(6), -G_o)), v_obj_star, np.dot(adg_config, f_spring_),
                                 np.full(16, np.inf)))
        constraints_all = LinearConstraint(J_all, cons_lb, cons_ub)

        Q = np.identity(24)
        Q[0:12,0:12] = np.identity(12)
        def cost(x):
            return np.dot(x.T,Q.dot(x))
        def jac_cost(x):
            return 2*Q.dot(x)

        # use quadprog
        Q = matrix(np.identity(24))
        p = matrix(np.zeros(24))
        A = matrix(np.concatenate((J1[6:],J2,J3)))
        b = matrix(np.concatenate((-G_o,v_obj_star,
                           np.dot(adg_config,f_spring_))))
        G = matrix(-J4)
        h = matrix(np.zeros(16))
        sol = solvers.qp(Q, p, G, h, A, b,options={'show_progress':False})
        print(sol['status']),
        print('solution after total iterations of'),
        print(sol['iterations'])
        return sol['x']

    def state_model(self,ut, x_robot_, x_config_, f_config_,x_obj_):
        # p_robot : the position of the rigid end
        # p_config: robot rigid end to the contact
        # return object position and contact force in the next timestep
        # pc: contact position relative to the object center
        # x_config: posistion + axis angle:  robot rigid end relative to the contact
        '''
        print('STATE ESTIMATION:')
        print('x_robot_', x_robot_)
        print('x_config_', x_config_)
        print('f_config_', f_config_)
        print('ref_obj_', x_obj_)
        '''
        p_robot_ = x_robot_[0:3]
        R_robot_ = quat2rotm(x_robot_[3:])
        p_obj_ = x_obj_[0:3]
        q_obj_ = x_obj_[3:]
        p_config_ = x_config_[0:3]
        R_config_ = exp2rotm(x_config_[3:])

        Rut= exp2rotm(ut[3:])
        p_robot = np.add(p_robot_, ut[0:3])
        #R_robot = np.dot(R_robot_,Rut)
        R_robot = np.dot(Rut,R_robot_)
        q_robot = rotm2quat(R_robot)
        q_config_ = rotm2quat(R_config_)

        K_config = self.vc_mapping.Kfx(x_config_.reshape(1, -1)) #force exert on the rigid end relative to rigid end pos change
        #K_config = -np.identity(6)*10
        K_spring = K_config
        f_spring = -f_config_
        #f_spring = np.ones(6)*0.1
        # solve for dp_obj, dq_obj, dp_config, dq_config
        #print('K_spring', K_spring)

        def robot_constraints(x):
            dp_obj = x[0:3]
            dq_obj = x[3:7]/np.linalg.norm(x[3:7])
            dp_config=x[7:10]
            dq_config=x[10:14]/np.linalg.norm(x[10:14])
            q_middle = quat_mul(q_obj_,self.qc)
            #eqn_q = quat_mul(quat_mul(quat_mul(dq_obj,q_middle),dq_config),q_config_) - q_robot
            q_temp = quat_mul(quat_mul(quat_mul(dq_obj, q_middle), dq_config), q_config_)
            eqn_q = np.dot(q_temp,q_robot) - 1
            R_obj = quat2rotm(dq_obj)*quat2rotm(q_obj_)
            eqn_p = np.linalg.norm(p_obj_ + dp_obj + np.dot(R_obj,self.pc) + R_obj.dot(self.Rc.dot(p_config_ + dp_config)) - p_robot)
            return np.concatenate(([eqn_q], [eqn_p]))

        def env_constraints(x):
            dp_obj = x[0:3]
            dq_obj = x[3:7]/np.linalg.norm(x[3:7])

            R_obj = np.dot(quat2rotm(dq_obj),quat2rotm(q_obj_))
            eqn1 = np.sum((np.dot(R_obj,self.envc1) + p_obj_ + dp_obj - self.envc1_w)**2)
            eqn2 = np.sum((np.dot(R_obj,self.envc2) + p_obj_ + dp_obj - self.envc2_w)**2)

            return np.concatenate(([eqn1],[eqn2]))

        def force_constraints(x):
            dp_obj = x[0:3]
            dq_obj = x[3:7]/np.linalg.norm(x[3:7])
            R_obj = np.dot(quat2rotm(dq_obj),quat2rotm(q_obj_))
            dp_config=x[7:10]
            dq_config=x[10:14]/np.linalg.norm(x[10:14])
            R_config = np.dot(quat2rotm(dq_config), quat2rotm(q_config_))
            p_config = dp_config+p_config_
            f_contact = np.dot(adjointTrans(R_config.T, -np.dot(R_config.T, 0.001 * p_config)), f_spring)

            f_co = np.dot(adjointTrans(self.Rc.T, -np.dot(self.Rc.T,self.pc)).T, f_contact.reshape(-1))
            G_o = np.concatenate((np.dot(R_obj.T,[0,0,-self.obj_m*self.gravity]),np.zeros(3)),axis = 0).T

            ADG_env = np.zeros([6,6])
            ADG_env[0:3,0:3] = R_obj.T
            ADG_env[0:3,3:] = R_obj.T
            ADG_env[3:,0:3] = np.dot(-skew_sym(self.envc1),R_obj.T)
            ADG_env[3:,3:] = np.dot(-skew_sym(self.envc2), R_obj.T)
            f_envo = np.dot(ADG_env,x[14:])
            #print('f_spring',f_spring)
            #print('f_contact',f_contact)
            #print('fco',f_co)
            #print('G',G_o)
            #print('fenv',f_envo)
            eqn_env_f = np.linalg.norm(f_co+G_o+f_envo)/100
            return eqn_env_f

        def energy(x):
            dp_obj = x[0:3]
            dq_obj = x[3:7]/np.linalg.norm(x[3:7])
            dp_config=x[7:10]
            dq_config=x[10:14]/np.linalg.norm(x[10:14])

            # f_config_: previous wrench on the robot rigid end
            x_ = np.concatenate((dp_config,quat2exp(dq_config)))
            #energy = self.obj_m*self.gravity*dp_obj[2] + 0.5*np.dot(x_,np.dot(K_spring,x_)) + np.dot(f_spring,x_)
            energy = 10*np.linalg.norm(dp_obj[0:3]) + 5*np.linalg.norm(np.dot(dq_obj,[1,0,0,0])-1) + 100*np.linalg.norm(dp_config[0:3]) + 10000*np.linalg.norm(np.dot(dq_config,[1,0,0,0])-1)
            return energy

        def cost(x):
            dp_config = x[7:10]
            dq_config = x[10:14] / np.linalg.norm(x[10:14])
            c= np.linalg.norm(dp_config[0:3])**2 + 10*np.linalg.norm(np.dot(dq_config, [1, 0, 0, 0]) - 1)
            return c

        def jac_cost(x):
            jac_c = np.zeros(x.shape)
            jac_c[7:10] = 2*x[7:10]
            jac_c[10] = 2*(x[10]/np.linalg.norm(x[10:14])-1)
            return jac_c

        np.random.seed(0)
        x0 = np.zeros(16)
        x0[0:3] = np.random.rand(3)/10
        x0[7:10] = np.random.rand(3)/10
        x0[3:7]= np.array([1,0,0,0])
        x0[10:14] =np.array([1,0,0,0])
        '''
        cons = ({"type": "eq", "fun": env_constraints},
                {"type": "eq", "fun": robot_constraints},
                {"type": "eq", "fun": force_constraints},
                {"type": "ineq", "fun": lambda x:  p_obj_[2] + x[2] - self.obj_lz},
                {"type": "eq", "fun": lambda x:  1 - np.dot(x[3:7],x[3:7])},
                {"type": "eq", "fun": lambda x:  1 - np.dot(x[10:14],x[10:14])},
                {"type": "ineq", "fun": lambda x: self.fric_mu * x[16] - np.linalg.norm(x[14:16])},
                {"type": "ineq", "fun": lambda x: self.fric_mu * x[19] - np.linalg.norm(x[17:19])})
        '''
        cons = ({"type": "eq", "fun": env_constraints},
                {"type": "eq", "fun": robot_constraints},
                {"type": "ineq", "fun": lambda x: p_obj_[2] + x[2] - self.obj_lz},
                {"type": "eq", "fun": lambda x: 1 - np.dot(x[3:7], x[3:7])},
                {"type": "eq", "fun": lambda x: 1 - np.dot(x[10:14], x[10:14])})
        #res = minimize(energy, x0, method='SLSQP', tol=1e-4, jac=jac_energy, constraints=cons)
        opts = {'maxiter':500}
        res = minimize(cost,x0,jac = jac_cost, tol=1e-3,constraints=cons,  options=opts)#, bounds = bnds)
        print('status:',res.status,', iter:',res.nit, ', fun:',res.fun)
        #print('x res', x0)
        #print('env_constraints res', env_constraints(res.x))
        #print('robot_constraints res', robot_constraints(res.x))
        if res.status == 0:
            return_x = res.x
        else:
            return_x = np.zeros(14)
            return_x[3:7]= np.array([1,0,0,0])
            return_x[10:14] =np.array([1,0,0,0])
        return return_x

    def linear_state_estimation(self, x_robot, x_robot_, x_config_, x_obj_, ft_force):
        # x: v_obj_body, v_config_body
        x_config_guess = self.vc_mapping.Mapping(-ft_force.reshape(1, -1), 'wrench', 'config').reshape(-1)
        dx_config_guess = x_config_guess - np.concatenate((x_config_[0:3],quat2exp(x_config_[3:])))
        R_robot = quat2rotm(x_robot[3:])
        R_robot_ = quat2rotm(x_robot_[3:])
        # compute robot spatial vel
        v_robot_spatial = np.concatenate((np.linalg.multi_dot([-R_robot,R_robot_.T,x_robot_[0:3]])+x_robot_[0:3],rotm2exp(np.dot(R_robot,R_robot_.T))))
        # compute robot constraint matrix: J_robot*x = v_robot_spatial
        J_robot = np.zeros((6,12))
        J_robot[0:6,0:6] = np.identity(6)
        J_robot[0:6,6:] = -np.dot(adjointTrans(quat2rotm(x_obj_[3:]), x_obj_[0:3]),adjointTrans(self.Rc, self.pc))
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
        G= np.concatenate((np.zeros(6),adg_obj[2])).reshape(1,-1)
        #bound = np.array([10,10,10,np.pi/2,np.pi/2,np.pi/2,10,10,10,np.pi/2,np.pi/2,np.pi/2])
        #h = np.concatenate((bound,bound))
        h = x_obj_[2] - self.obj_lz
        sol = solvers.qp(matrix(P), matrix(p), matrix(G), matrix(h), matrix(A), matrix(b),options={'show_progress':False})
        print(sol['status']),
        print('solution after total iterations of'),
        print(sol['iterations'])
        return np.array(sol['x']).reshape(-1)

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
        Xref, Xref_nbrs, Yref, alpha = self.__get_param(X_name,Y_name)
        X_ = X*alpha
        distances, indices = Xref_nbrs.kneighbors(X_)
        #itself = np.argwhere(distances==0)
        #if len(itself)!=0:
        #    indices = np.delete(indices,np.argwhere(distances==0)[0])
        X_ind = np.squeeze(Xref[indices])
        Y_ind = np.squeeze(Yref[indices])

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
        r = np.linalg.norm(x[:,:-1]-x0[:-1])
        if r > 1.2*r_max:
            x = x*1.2*r_max/r
        y_predict = np.dot(K, (x - x0).T).reshape(-1) + y0

        return y_predict