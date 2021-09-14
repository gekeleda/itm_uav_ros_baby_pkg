#!/usr/bin/env python3
import rospy
import numpy as np
from itm_mav_msgs.msg import itm_trajectory_msg
from itm_mav_msgs.msg import itm_trajectory_point
from std_msgs.msg import Header

delay_init = 150
delay_traj = 275
n_nodes = 20

dt = 0.1
dt_rate = 0.01

reference_point = np.array([0.0, 0.0, 1.0, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.])

vel = 0.4
veliter = vel/n_nodes
dist = np.linalg.norm(reference_point[:6])
steps = int(dist/veliter)

def trajectory_generator(iter_n, current_trajectory, traj_shape='circle'):
    traj_fac = 30 # 30

    if traj_shape=='spiral':
        kz = 0.0013
        next_trajectories = current_trajectory[1:, :]
        # next_trajectories = np.concatenate((next_trajectories,
        # np.array([np.cos((iter_n)/traj_fac), np.sin((iter_n)/traj_fac), 1.0, 0.0, 0.0, 0.0, 0., 0., 0.0, 0.0, 0.0, 0.0, 0.0]).reshape(1, -1)))
        next_trajectories = np.concatenate((next_trajectories,
        np.array([np.cos((iter_n)/traj_fac)-1, np.sin((iter_n)/traj_fac), 1.0+kz*iter_n, -np.sin(iter_n/traj_fac)/(dt*traj_fac), np.cos(iter_n/traj_fac)/(dt*traj_fac), 0.0, 0., 0., 0.0, 0.0, 0.0, 0.0, 0.0]).reshape(1, -1)))
        return next_trajectories
    elif traj_shape=='eight':
        next_trajectories = current_trajectory[1:, :]
        # next_trajectories = np.concatenate((next_trajectories,
        # np.array([np.cos((iter_n)/traj_fac), np.sin((iter_n)/traj_fac), 1.0, 0.0, 0.0, 0.0, 0., 0., 0.0, 0.0, 0.0, 0.0, 0.0]).reshape(1, -1)))
        next_trajectories = np.concatenate((next_trajectories,
        np.array([np.cos((iter_n)/traj_fac)-1, 0.5*np.sin(2*(iter_n)/traj_fac), 1.0, -np.sin(iter_n/traj_fac)/(dt*traj_fac), np.cos(2*iter_n/traj_fac)/(2*dt*traj_fac), 0.0, 0., 0., 0.0, 0.0, 0.0, 0.0, 0.0]).reshape(1, -1)))
        return next_trajectories

    next_trajectories = current_trajectory[1:, :]
    # next_trajectories = np.concatenate((next_trajectories,
    # np.array([np.cos((iter_n)/traj_fac), np.sin((iter_n)/traj_fac), 1.0, 0.0, 0.0, 0.0, 0., 0., 0.0, 0.0, 0.0, 0.0, 0.0]).reshape(1, -1)))
    next_trajectories = np.concatenate((next_trajectories,
    np.array([np.cos((iter_n)/traj_fac)-1, np.sin((iter_n)/traj_fac), 1.0, -np.sin(iter_n/traj_fac)/(dt*traj_fac), np.cos(iter_n/traj_fac)/(dt*traj_fac), 0.0, 0., 0., 0.0, 0.0, 0.0, 0.0, 0.0]).reshape(1, -1)))
    return next_trajectories


def makePoint(reference_point):
    point = itm_trajectory_point()
    point.x = reference_point[0]
    point.y = reference_point[1]
    point.z = reference_point[2]
    point.vx = reference_point[3]
    point.vy = reference_point[4]
    point.vz = reference_point[5]
    point.load_x = reference_point[6]
    point.load_y = reference_point[7]
    point.load_vx = reference_point[8]
    point.load_vy = reference_point[9]
    point.roll = reference_point[10]
    point.pitch = reference_point[11]
    point.yaw = reference_point[12]
    return point


def makeMsg(trajectory):
    msg = itm_trajectory_msg()
    msg.header = Header()
    msg.size = n_nodes
    msg.data = []
    msg.traj = []
    for ref_point in trajectory:
        point = makePoint(ref_point)
        msg.traj.append(point)
    return msg

def getTrajectory(i, k, start_point, old_trajectory):
    if i < delay_traj:
        next_trajectories = old_trajectory[1:, :]
        fac = min((i+k-delay_init)/steps, 1.)
        next_trajectories = np.concatenate((next_trajectories, (fac*reference_point + (1-fac)*start_point).reshape(1, -1)))
        return next_trajectories
    else:
        return trajectory_generator(i-delay_traj, old_trajectory, traj_shape="circle") #shape: circle, spiral or eight


def talker():
    pub = rospy.Publisher('/robot_trajectory',
                          itm_trajectory_msg, queue_size=1)
    rospy.init_node('trajectory_node')
    fac_dt = dt/dt_rate
    rate = rospy.Rate(1/dt_rate)

    start_point = np.zeros_like(reference_point)
    # start_point[2] = 0.5
    trajectory = np.array([start_point]*n_nodes)

    i = 0
    j = 0
    while not rospy.is_shutdown():
        if i%fac_dt==0:
            # rospy.loginfo(msg)
            if j == delay_init:
                for k in range(n_nodes):
                    trajectory[k] = k/steps *reference_point + (1-k/steps)*start_point

            if j >= delay_init:
                trajectory = getTrajectory(j, n_nodes, start_point, trajectory)

            j += 1

        msg = makeMsg(trajectory)
        pub.publish(msg)
        i += 1
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
