#!/usr/bin/env python

'''
Author: Wei Luo
Date: 2020-09-29 16:44:26
LastEditors: Wei Luo
LastEditTime: 2021-04-25 23:46:01
Note: Note
'''

import rospy
import mavros
import time
# from uav_core import mavros_core_function
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import CommandBool, SetMode
from std_srvs.srv import SetBool, SetBoolResponse
from geometry_msgs.msg import PoseStamped
from pymavlink import mavutil
PKG = 'px4'
class offboard_controller():
    def __init__(self, controller_name='mpc'):
        # super(offboard_controller, self).__init__() # only workable for python3
        # mavros_core_function.__init__(self, ) # handle default mavros info
        self.last_request = None
        self.state = State()
        self.extended_state = ExtendedState()

        ## Topics
        self.local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        rospy.Subscriber('/mavros/state', State, self.mavros_state_callback)
        self.ext_state_sub = rospy.Subscriber('mavros/extended_state',
                                              ExtendedState,
                                              self.extended_state_callback)

        ## Service
        self.set_arming_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        ## Timer
        self.rate = rospy.Rate(50)


        if controller_name == 'pid':
            rospy.wait_for_service('uav_pid_server')
            self.control_state_check_function = rospy.ServiceProxy('uav_pid_server', SetBool)
            self.controller_state = self.control_state_check_function(True)
            # print(self.controller_state)
        elif controller_name == 'mpc':
            rospy.wait_for_service('uav_mpc_server')
            self.control_state_check_function = rospy.ServiceProxy('uav_mpc_server', SetBool)
            self.controller_state = self.control_state_check_function(True)
        else:
            rospy.info("Unknown controller")

        # self.set_local_position()
        if self.controller_state.success:
            # setup the arm and ready for the flight
            # self.set_arm(True, 10)
            self.offboard_ready = True
        else:
            rospy.loginfo('UAV controller server is not ready yet, please launch a controller first')
            self.offboard_ready = False


    def mavros_state_callback(self, data):
        # Print armed state change
        if self.state.armed != data.armed:
            rospy.loginfo('UAV: ' + "armed state changed from {0} to {1}".format(self.state.armed, data.armed))

        # Print connection state change
        if self.state.connected != data.connected:
            rospy.loginfo('UAV: ' + "connected changed from {0} to {1}".format(self.state.connected, data.connected))

        # Print mode state change
        if self.state.mode != data.mode:
            rospy.loginfo('UAV: ' + "mode changed from {0} to {1}".format(self.state.mode, data.mode))

        # Print system status change
        if self.state.system_status != data.system_status:
            rospy.loginfo('UAV: ' + "system_status changed from {0} to {1}".format(mavutil.mavlink.enums['MAV_STATE'][self.state.system_status].name, mavutil.mavlink.enums['MAV_STATE'][data.system_status].name))

        self.state = data

        # mavros publishes a disconnected state message on init
        # Set topic 'state' as ready
        # if not self.sub_topics_ready['state'] and data.connected:
        #     self.sub_topics_ready['state'] = True

    def extended_state_callback(self, data):
        if self.extended_state.vtol_state != data.vtol_state:
            rospy.loginfo("VTOL state changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_VTOL_STATE']
                [self.extended_state.vtol_state].name, mavutil.mavlink.enums[
                    'MAV_VTOL_STATE'][data.vtol_state].name))

        if self.extended_state.landed_state != data.landed_state:
            rospy.loginfo("landed state changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_LANDED_STATE']
                [self.extended_state.landed_state].name, mavutil.mavlink.enums[
                    'MAV_LANDED_STATE'][data.landed_state].name))

        self.extended_state = data

    def guard(self,):
        if self.controller_state.success:
            if not self.state.mode == "OFFBOARD" and rospy.Time.now() - self.last_request > rospy.Duration.from_sec(5.0):
                offboard_res = self.set_mode_srv(0, "OFFBOARD")
                # arm_res = self.set_arming_srv(True)
                if offboard_res.mode_sent:
                    rospy.loginfo_once("Offboard enabled")
                    self.last_request = rospy.Time.now()
            else:
                if not self.state.armed and rospy.Time.now() - self.last_request > rospy.Duration.from_sec(5.0):
                    print('try to arm')
                    arm_res = self.set_arming_srv(True)
                    if arm_res:
                        rospy.loginfo_once('Arm the UAV')
                        self.last_request = rospy.Time.now()
        else:
            offboard_res = self.set_mode_srv(0, "AUTO.LAND")
            if offboard_res.mode_sent:
                rospy.loginfo_once("Landing Now")
            if self.extended_state.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:

                self.offboard_ready = False

        #     self.set_mode('OFFBOARD', 10)
        # else:
        #     pass
        # print('guard')
        try:
            self.controller_state = self.control_state_check_function(True)
        except:

            self.controller_state.success = False
            # print('The controller is lost. Now is to land immediately')




if __name__ == '__main__':
    rospy.init_node('offboard_controller')
    controller_mode = rospy.get_param('~controller', 'mpc')
    offboard_obj = offboard_controller(controller_name=controller_mode)
    offboard_obj.last_request = rospy.Time.now()
    while not rospy.is_shutdown() and offboard_obj.offboard_ready:
        # guard function to maintain offboard avaliable
        offboard_obj.guard()
        offboard_obj.rate.sleep()

    # stop the offboard mode
    print("offboard control is stopped")

