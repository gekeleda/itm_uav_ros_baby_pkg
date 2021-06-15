#!/usr/bin/env python3
import rospy
import numpy as np
from itm_mav_msgs.msg import itm_trajectory_msg
from itm_mav_msgs.msg import itm_trajectory_point
from std_msgs.msg import Header

l = 0.1

delay_init = 100
delay_traj = 200
n_nodes = 20

reference_point = np.array([1.0, 0., 1.0, 0., 0., 0., 1.0, 0., 1.0-l, 0., 0., 0., 0.])

vel = 0.6
veliter = vel/n_nodes
dist = np.linalg.norm(reference_point[:6])
steps = int(dist/veliter)

def trajectory_generator(iter, current_trajectory):
    traj_k = 30 # 30
    next_trajectories = current_trajectory[1:, :]
    next_trajectories = np.concatenate((next_trajectories,
    np.array([np.cos((iter)/traj_k), np.sin((iter)/traj_k), 1.0, 0.0, 0.0, 0.0, np.cos((iter)/traj_k), np.sin((iter)/traj_k), 1.0-l, 0.0, 0.0, 0.0, 0.0]).reshape(1, -1)))
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

def getTrajectory(i, old_trajectory):
    if i < delay_traj:
        next_trajectories = old_trajectory[1:, :]
        fac = min((i-delay_init)/steps, 1.)
        next_trajectories = np.concatenate((next_trajectories, (fac*reference_point).reshape(1, -1)))
        return next_trajectories
    else:
        return trajectory_generator(i-delay_traj, old_trajectory)


def talker():
    pub = rospy.Publisher('/robot_trajectory', itm_trajectory_msg, queue_size=1)
    rospy.init_node('trajectory_node')
    dt = 0.1
    rate = rospy.Rate(1/dt)

    trajectory = np.array([np.zeros_like(reference_point)]*n_nodes)

    i = 0
    while not rospy.is_shutdown():
        #rospy.loginfo(msg)
        if i == delay_init:
            for k in range(n_nodes):
                trajectory[k] = k/steps * reference_point

        if i >= delay_init:
            trajectory = getTrajectory(i, trajectory)

        msg = makeMsg(trajectory)
        pub.publish(msg)
        i+=1
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass