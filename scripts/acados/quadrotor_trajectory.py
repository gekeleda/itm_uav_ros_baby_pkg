#!/usr/bin/env python3
import rospy
import numpy as np
from itm_mav_msgs.msg import itm_trajectory_msg
from itm_mav_msgs.msg import itm_trajectory_point
from std_msgs.msg import Header

reference_point = np.array([0.5, 0., 1.0, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.])
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
    msg.traj = [point]*n_nodes

    while not rospy.is_shutdown():
        #rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass