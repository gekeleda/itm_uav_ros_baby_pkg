#!/usr/bin/env python3
import rospy
import numpy as np
from itm_mav_msgs.msg import itm_trajectory_msg
from itm_mav_msgs.msg import itm_trajectory_point
from std_msgs.msg import Header

reference_point = np.array([0., 0., 10., 0., 0., 0., 0., 0., 0.])
n_nodes = 20

def talker():
    pub = rospy.Publisher('/robot_trajectory', itm_trajectory_msg, queue_size=1)
    rospy.init_node('trajectory_node')
    dt = 0.1
    rate = rospy.Rate(1/dt)

    msg = itm_trajectory_msg()
    msg.header = Header()
    msg.size = n_nodes
    msg.data = []
    point = itm_trajectory_point()
    point.x = 0.
    point.y = 0.
    point.z = 10.
    point.vx = 0.
    point.vy = 0.
    point.vz = 0.
    point.roll = 0.
    point.pitch = 0.
    point.yaw = 0.
    msg.traj = [point]*n_nodes

    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass