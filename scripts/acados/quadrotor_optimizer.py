#!/usr/bin/env python3
# coding=UTF-8
'''
Author: Wei Luo
Date: 2021-03-21 22:27:50
LastEditors: Wei Luo
LastEditTime: 2021-06-10 23:36:29
Note: Note
'''
import os
import sys
# from utils.utils import safe_mkdir_recursive
from quadrotor_model import QuadRotorModel
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
import casadi as ca
import scipy.linalg
import numpy as np
import time
import threading

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from itm_mav_msgs.msg import itm_trajectory_msg
from std_srvs.srv import SetBool, SetBoolResponse
from mavros_msgs.msg import AttitudeTarget
from geometry_msgs.msg import Quaternion
from threading import Thread
from std_msgs.msg import Header
from scipy.ndimage.filters import gaussian_filter1d

import matplotlib.pyplot as plt


class QuadOptimizer:
    def __init__(self, quad_model, quad_constraints, t_horizon, n_nodes, sim_required=False, max_hight=1.0):
        self.model = quad_model
        self.constraints = quad_constraints
        self.g = 9.8066
        self.T = t_horizon
        self.N = n_nodes
        self.simulation_required = sim_required

        robot_name_ = rospy.get_param("~robot_name", "bebop1_r")
        self.current_pose = None
        self.current_state = np.zeros((13, 1))
        self.dt = 0.02
        self.rate = rospy.Rate(1/self.dt)
        self.time_stamp = None
        self.trajectory_path = None
        self.current_twist = np.zeros(3)
        self.att_command = AttitudeTarget()
        self.att_command.type_mask = 3
        self.pendulum_state = np.zeros(4)

        # subscribers
        # the robot state
        robot_state_sub_ = rospy.Subscriber(
            '/robot_pose', Odometry, self.robot_state_callback)
        # pendulum state
        pendulum_state_sub_ = rospy.Subscriber('/pendulum_pose', Odometry, self.pendulum_state_callback)
        # trajectory
        robot_trajectory_sub_ = rospy.Subscriber(
            '/robot_trajectory', itm_trajectory_msg, self.trajectory_command_callback)
        # publisher
        self.att_setpoint_pub = rospy.Publisher(
            '/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
        # create a server
        server_ = rospy.Service('uav_mpc_server', SetBool, self.state_server)

        # setup optimizer
        self.quadrotor_optimizer_setup()

        # # It seems that thread cannot ensure the performance of the time
        self.att_thread = Thread(target=self.send_command, args=())
        self.att_thread.daemon = True
        self.att_thread.start()

    def robot_state_callback(self, data):
        # robot state as [x, y, z, vx, vy, vz, roll, pitch, yaw]
        roll, pitch, yaw = self.quaternion_to_rpy(data.pose.pose.orientation)

        # self.current_state[:6] = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z,
        #                                 data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z]).reshape(6,1)
        # self.current_state[-3:] = np.array([roll, pitch, yaw]).reshape(3,1)

        self.current_state = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z,
                                       data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z,
                                       data.pose.pose.position.x, data.pose.pose.position.y, 0., 0.,
                                       roll, pitch, yaw], dtype=np.float64)

    def pendulum_state_callback(self, data):
        # pendulum state as [x, y, z, vx, vy, vz, s, r, ds, dr, roll, pitch, yaw]
        s, r = data.pose.pose.position.x, data.pose.pose.position.y
        ds, dr = data.twist.twist.linear.x, data.twist.twist.linear.y
        # self.current_state[7:10] = np.array([s, r, ds, dr]).reshape(4, 1)
        self.pendulum_state = np.array([s, r, ds, dr])  # .reshape(4, 1)

        # self.current_state_pendulum = np.array([self.current_state[0], self.current_state[1], self.current_state[2],
        #                                 self.current_state[3], self.current_state[4], self.current_state[5],
        #                                 s, r, ds, dr,
        #                                 self.current_state[10], self.current_state[11], self.current_state[12]], dtype=np.float64)

    def trajectory_command_callback(self, data):
        temp_traj = data.traj
        if data.size != len(temp_traj):
            rospy.logerr('Some data are lost')
        else:
            self.trajectory_path = np.zeros((data.size, 13))
            for i in range(data.size):
                self.trajectory_path[i] = np.array([temp_traj[i].x,
                                                    temp_traj[i].y,
                                                    temp_traj[i].z,
                                                    temp_traj[i].vx,
                                                    temp_traj[i].vy,
                                                    temp_traj[i].vz,
                                                    temp_traj[i].load_x,
                                                    temp_traj[i].load_y,
                                                    temp_traj[i].load_vx,
                                                    temp_traj[i].load_vy,
                                                    temp_traj[i].roll,
                                                    temp_traj[i].pitch,
                                                    temp_traj[i].yaw,
                                                    ])

    def quadrotor_optimizer_setup(self, ):
        # Q_m_ = np.diag([80.0, 80.0, 120.0, 20.0, 20.0,
        #                 30.0, 10.0, 10.0, 0.0])  # position, velocity, roll, pitch, (not yaw)

        # P_m_ = np.diag([86.21, 86.21, 120.95,
        #                 6.94, 6.94, 11.04])  # only p and v
        # P_m_[0, 3] = 6.45
        # P_m_[3, 0] = 6.45
        # P_m_[1, 4] = 6.45
        # P_m_[4, 1] = 6.45
        # P_m_[2, 5] = 10.95
        # P_m_[5, 2] = 10.95
        # R_m_ = np.diag([50.0, 60.0, 1.0])
        Q_m_ = np.diag([10.0, 10.0, 10.0,
                        3e-1, 3e-1, 3e-1,
                        #3e-1, 3e-1, 3e-2, 3e-2,
                        100.0, 100.0, 1e-3, 1e-3,
                        10.5, 10.5, 10.5])  # position, velocity, load_position, load_velocity, [roll, pitch, yaw]

        P_m_ = np.diag([10.0, 10.0, 10.0,
                        0.05, 0.05, 0.05
                        # 10.0, 10.0, 10.0,
                        # 0.05, 0.05, 0.05
                        ])  # only p and v
        # P_m_[0, 8] = 6.45
        # P_m_[8, 0] = 6.45
        # P_m_[1, 9] = 6.45
        # P_m_[9, 1] = 6.45
        # P_m_[2, 10] = 10.95
        # P_m_[10, 2] = 10.95
        R_m_ = np.diag([3.0, 3.0, 3.0, 1.0])

        nx = self.model.x.size()[0]
        self.nx = nx
        nu = self.model.u.size()[0]
        self.nu = nu
        ny = nx + nu
        n_params = self.model.p.size()[0] if isinstance(
            self.model.p, ca.SX) else 0

        acados_source_path = os.environ['ACADOS_SOURCE_DIR']
        sys.path.insert(0, acados_source_path)

        # create OCP
        ocp = AcadosOcp()
        ocp.acados_include_path = acados_source_path + '/include'
        ocp.acados_lib_path = acados_source_path + '/lib'
        ocp.model = self.model
        ocp.dims.N = self.N
        ocp.solver_options.tf = self.T

        # initialize parameters
        ocp.dims.np = n_params
        ocp.parameter_values = np.zeros(n_params)

        # cost type
        ocp.cost.cost_type = 'LINEAR_LS'
        ocp.cost.cost_type_e = 'LINEAR_LS'
        ocp.cost.W = scipy.linalg.block_diag(Q_m_, R_m_)
        ocp.cost.W_e = P_m_  # np.zeros((nx-3, nx-3))

        ocp.cost.Vx = np.zeros((ny, nx))
        ocp.cost.Vx[:nx, :nx] = np.eye(nx)
        ocp.cost.Vu = np.zeros((ny, nu))
        ocp.cost.Vu[-nu:, -nu:] = np.eye(nu)
        ocp.cost.Vx_e = np.zeros((nx-7, nx))  # only consider p and v
        ocp.cost.Vx_e[:nx-7, :nx-7] = np.eye(nx-7)

        # initial reference trajectory_ref
        x_ref = np.zeros(nx)
        x_ref_e = np.zeros(nx-7)
        u_ref = np.zeros(nu)
        u_ref[-1] = self.g
        ocp.cost.yref = np.concatenate((x_ref, u_ref))
        ocp.cost.yref_e = x_ref_e

        # Set constraints
        ocp.constraints.lbu = np.array(
            [self.constraints.roll_min, self.constraints.pitch_min, self.constraints.yaw_min, self.constraints.thrust_min])
        ocp.constraints.ubu = np.array(
            [self.constraints.roll_max, self.constraints.pitch_max, self.constraints.yaw_max, self.constraints.thrust_max])
        ocp.constraints.idxbu = np.array([0, 1, 2, 3])

        # initial state
        ocp.constraints.x0 = x_ref

        # solver options
        ocp.solver_options.qp_solver = 'FULL_CONDENSING_HPIPM'
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        # explicit Runge-Kutta integrator
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.print_level = 0
        ocp.solver_options.nlp_solver_type = 'SQP'  # 'SQP_RTI'

        ocp.solver_options.levenberg_marquardt = 0.12 # 0.0

        # compile acados ocp
        json_file = os.path.join('./'+self.model.name+'_acados_ocp.json')
        self.solver = AcadosOcpSolver(ocp, json_file=json_file)
        if self.simulation_required:
            self.integrator = AcadosSimSolver(ocp, json_file=json_file)

    def mpc_estimation_loop(self, mpc_iter):
        if self.trajectory_path is not None and self.current_state is not None:
            t1 = time.time()
            # dt = 0.1
            current_trajectory = self.trajectory_path

            u_des = np.array([0.0, 0.0, 0.0, self.g])
            new_state = self.current_state
            # new_state[6:10] = self.pendulum_state
            new_state[6] = self.pendulum_state[0]-self.current_state[0]
            new_state[7] = self.pendulum_state[1]-self.current_state[1]
            new_state[8] = self.pendulum_state[2]-self.current_state[3]
            new_state[9] = self.pendulum_state[3]-self.current_state[4]

            simX[mpc_iter] = new_state # mpc_iter+1 ?
            simD[mpc_iter] = current_trajectory[0]

            l = 0.1
            #print(np.rad2deg(np.arcsin(new_state[6]/l)))

            # print()
            # print("current state: ")
            # np.set_printoptions(suppress=True)
            # print(new_state)

            # print()
            # print("current trajectory: ")
            # print(current_trajectory[-1])

            tra = current_trajectory[0]
            # error_xyz = np.linalg.norm(np.array([new_state[0] - tra[0], new_state[1] - tra[1], new_state[2] - tra[2]]))
            # print(error_xyz)

            self.solver.set(self.N, 'yref', current_trajectory[-1, :6])
            for i in range(self.N):
                self.solver.set(i, 'yref', np.concatenate(
                    [current_trajectory[i].flatten(), u_des]))

            # self.solver.set(0, 'lbx', self.current_state)
            # self.solver.set(0, 'ubx', self.current_state)
            self.solver.set(0, 'lbx', new_state)
            self.solver.set(0, 'ubx', new_state)

            status = self.solver.solve()

            tarr[mpc_iter] = time.time() - t1

            if status != 0:
                rospy.logerr("MPC cannot find a proper solution.")
                # self.solver.print_statistics()
                print()
                print("current state: ")
                np.set_printoptions(suppress=True)
                print(new_state)

                print()
                print("current trajectory: ")
                print(current_trajectory[0])


                self.att_command.orientation = Quaternion(
                    *self.rpy_to_quaternion(0.0, 0.0, 0.0, w_first=False))
                #self.att_command.thrust = 0.5
                self.att_command.thrust = 0.62
                self.att_command.body_rate.z = 0.0
            else:
                # print()
                # print("predicted trajectory:")
                # for i in range(self.N):
                #     print(self.solver.get(i, 'x'))
                mpc_u_ = self.solver.get(0, 'u')
                simU[mpc_iter] = mpc_u_
                quat_local_ = self.rpy_to_quaternion(
                    mpc_u_[0], mpc_u_[1], mpc_u_[2], w_first=False)
                self.att_command.orientation.x = quat_local_[0]
                self.att_command.orientation.y = quat_local_[1]
                self.att_command.orientation.z = quat_local_[2]
                self.att_command.orientation.w = quat_local_[3]
                # print(self.att_command.orientation)
                #self.att_command.thrust = mpc_u_[3]/9.8066 - 0.5
                # self.att_command.thrust = mpc_u_[3]/9.8066 - 0.38
                self.att_command.thrust = mpc_u_[3]/9.8066*0.6175
                # print()
                # print("mpc_u: ")
                # print(mpc_u_)
                # print()
                # print("---------------------------------------")
                # print(self.att_command.thrust)
                # yaw_command_ = self.yaw_command(current_yaw_, trajectory_path_[1, -1], 0.0)
                # yaw_command_ = self.yaw_controller(trajectory_path_[1, -1]-current_yaw_)
                # self.att_command.angular.z = yaw_command_
                #self.att_command.body_rate.z = 0.0

            # self.att_setpoint_pub.publish(self.att_command)
            # print("time: ")
            # print(time.time()-time_1)

        else:
            if self.trajectory_path is None:
                rospy.loginfo("waiting trajectory")
            elif self.current_state is None:
                rospy.loginfo("waiting current state")
            else:
                rospy.loginfo("Unknown error")
        self.rate.sleep()
        return True

    @staticmethod
    def quaternion_to_rpy(quaternion):
        q0, q1, q2, q3 = quaternion.w, quaternion.x, quaternion.y, quaternion.z
        roll_ = np.arctan2(2*(q0*q1+q2*q3), 1-2*(q1**2+q2**2))
        pitch_ = np.arcsin(2*(q0*q2-q3*q1))
        yaw_ = np.arctan2(2*(q0*q3+q1*q2), 1-2*(q2**2+q3**2))
        return roll_, pitch_, yaw_

    @staticmethod
    def state_server(req):
        return SetBoolResponse(True, 'MPC is ready')

    @staticmethod
    def rpy_to_quaternion(r, p, y, w_first=True):
        cy = np.cos(y*0.5)
        sy = np.sin(y*0.5)
        cp = np.cos(p*0.5)
        sp = np.sin(p*0.5)
        cr = np.cos(r*0.5)
        sr = np.sin(r*0.5)

        qw = cr*cp*cy + sr*sp*sy
        qx = sr*cp*cy - cr*sp*sy
        qy = cr*sp*cy + sr*cp*sy
        qz = cr*cp*sy - sr*sp*cy
        if w_first:
            return np.array([qw, qx, qy, qz])
        else:
            return np.array([qx, qy, qz, qw])

    def send_command(self,):
        rate = rospy.Rate(100)  # Hz
        self.att_command.header = Header()

        while not rospy.is_shutdown():
            # t2 = time.time()
            command_ = self.att_command
            # self.att_command.header.stamp = rospy.Time.now()
            self.att_setpoint_pub.publish(command_)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass
            # print("publsich loop takes {} seconds".format(time.time() - t2))


def smoothPlot(x, y, ax, lab):
    ysmoothed = gaussian_filter1d(y, sigma=8)
    ax.plot(x, ysmoothed, label=lab)

def plotPaths(simX, simD, mpc_iter, range_min=0, range_max=-1):
    if range_max==-1:
        range_max = mpc_iter
    l = 0.1
    hX = np.sqrt(l*l - np.square(simX[:mpc_iter,6]) - np.square(simX[:mpc_iter, 7]))
    hD = np.sqrt(l*l - np.square(simD[:mpc_iter,6]) - np.square(simD[:mpc_iter, 7]))
    loadX = np.array([simX[:mpc_iter, 0]+simX[:mpc_iter, 6], simX[:mpc_iter,1]+simX[:mpc_iter, 7], simX[:mpc_iter, 2]-hX])
    loadD = np.array([simD[:mpc_iter, 0]+simD[:mpc_iter, 6], simD[:mpc_iter,1]+simD[:mpc_iter, 7], simD[:mpc_iter, 2]-hD])

    alphaX = np.rad2deg(np.arcsin(simX[:mpc_iter, 6] / l))
    alphaD = np.rad2deg(np.arcsin(simD[:mpc_iter, 6] / l))

    print(np.max(np.abs(alphaX)))

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.plot(simX[:mpc_iter, 0], simX[:mpc_iter, 1], simX[:mpc_iter, 2], 'b', label="real quadcopter trajectory")
    ax.plot(simD[:mpc_iter, 0], simD[:mpc_iter, 1], simD[:mpc_iter, 2], 'b--', label="desired quadcopter trajectory")
    ax.plot(loadX[0], loadX[1], loadX[2], 'r', label="real pendulum trajectory")
    
    ax.legend(loc=(0.2, 0.3))
    plt.xlabel('[m]')
    plt.ylabel('[m]')
    ax.set_zlabel('[m]', rotation=0)
    # ax.plot(loadD[0], loadD[1], loadD[2], 'r--')

    iters = range(range_min, range_max)
    dt = 0.02
    t = np.linspace(0, dt*(range_max-range_min), num=range_max-range_min)

    plt.savefig('traj.png', bbox_inches='tight')
    plt.show()

    fig = plt.figure()
    ax = fig.gca()
    ax.plot(t, simX[iters, 0], label="real quadcopter x-coordinate")
    ax.plot(t, simD[iters, 0], '--', label="desired quadcopter x-coordinate")
    #plt.title("x coordinate of quadcopter")
    plt.xlabel("time in s")
    plt.ylabel("distance in m")
    #ax.legend()
    plt.savefig('res_x.png', bbox_inches='tight')
    plt.show()

    fig = plt.figure()
    ax = fig.gca()
    # ax.plot(iters, loadX[0][iters], )
    # ax.plot(iters, loadD[0][iters], )
    smoothPlot(t, simX[iters, 6], ax, "real pendulum x-coordinate")
    ax.plot(t, simD[iters, 6], '--', label="desired pendulum x-coordinate")
    #plt.title("x coordinate of pendulum, relative to quadcopter (smoothed)")
    plt.ylabel("distance in m")
    plt.xlabel("time in s")
    #ax.legend()
    plt.savefig('res_load_x.png', bbox_inches='tight')
    plt.show()

    fig = plt.figure()
    ax = fig.gca()
    smoothPlot(t, alphaX[iters], ax, "real pendulum angle")
    ax.plot(t, alphaD[iters], '--', label="desired pendulum angle")
    #plt.title("alpha angle of pendulum (smoothed)")
    #ax.legend()
    plt.ylabel("degrees")
    plt.xlabel("time in s")
    plt.savefig('res_load_angle.png', bbox_inches='tight')
    plt.show()

if __name__ == '__main__':
    print(os.path.dirname(os.path.realpath(__file__)))

    rospy.init_node('offboard_mpc_controller')

    sim_time = 95 # s
    dt = 0.02 # s
    nx = 13
    nu = 4

    simX = np.zeros((int(sim_time/dt+1), nx))
    simX[0] = np.array([0.0, 0.0, 0.0, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.])
    simD = np.zeros((int(sim_time/dt+1), nx))
    simU = np.zeros((int(sim_time/dt), nu))

    tarr = np.zeros((int(sim_time/dt+1), 1))

    quad_rotor_model = QuadRotorModel()
    try:
        mpc_obj = QuadOptimizer(quad_model=quad_rotor_model.model,
                                quad_constraints=quad_rotor_model.constraints,
                                t_horizon=2.,
                                n_nodes=20
                                )
        mpc_model_is_ready = True
    except ImportError:
        rospy.logerr('Cannot find any MPC library, Stop the node')
        mpc_model_is_ready = False
        mpc_obj = None

    time.sleep(2)

    main_iter = -1
    while not rospy.is_shutdown() and mpc_model_is_ready and main_iter<sim_time/dt-1:
        main_iter += 1
        if not mpc_obj.mpc_estimation_loop(main_iter):
            rospy.logerr("MPC estimation failed")
        # if main_iter%100==0:
        #     print(main_iter)
    
    print('MPC controller is shutdown')
    print('average time is {}'.format(np.sum(tarr)/main_iter))
    path = os.path.dirname(os.path.realpath(__file__))

    np.save(path + "/simX.npy", simX[:main_iter])
    np.save(path + "/simD.npy", simD[:main_iter])
    # np.save(path + "/simU.npy", simU[main_iter])

    plotPaths(simX, simD, main_iter, 1250, main_iter)