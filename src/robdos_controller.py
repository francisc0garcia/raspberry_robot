#!/usr/bin/env python

import roslib
import roslib
import sys
import rospy
import math
import time
import numpy as np
from mavros_msgs.msg import WaypointList, Waypoint, OverrideRCIn
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from dynamic_reconfigure.server import Server
from robdos_autonomous.cfg import controllerConfig
from robdos_state_machine.msg import StateEvent
from std_srvs.srv import Empty

from PID import *

class RobdosController:
    def __init__(self, debug=False):

        rospy.init_node('robdos_controller')

        self.degrees2rad = math.pi / 180.0
        self.rad2degrees = 180.0 / math.pi
        self.debug = debug
        self.current_target = None
        self.is_target_ready = False
        self.is_localization_ready = False

        # define rate of  100 hz
        self.rate = rospy.Rate(50.0)

        # init variables for odometry
        [self.robot_position_x, self.robot_position_y, self.robot_position_z] = [0, 0, 0]
        [self.robot_roll, self.robot_pitch, self.robot_yaw] = [0, 0, 0]

        # init control variables
        [self.threshold_position, self.threshold_orientation, self.threshold_depth] = [0.1, 10, 0.1]

        [self.P_kp, self.P_kd, self.P_ki, self.P_windup] = [0.0, 0.0, 0.0, 0.0]
        [self.O_kp, self.O_kd, self.O_ki, self.O_windup] = [0.0, 0.0, 0.0, 0.0]
        [self.D_kp, self.D_kd, self.D_ki, self.D_windup] = [0.0, 0.0, 0.0, 0.0]

        self.position_pid = PID()
        self.orientation_pid = PID()
        self.depth_pid = PID()

        [self.limit_min, self.limit_max] = [0.0, 0.0]

        """initialize position and orientation controller"""
        self.srv = Server(controllerConfig, self.reconfig_callback)

        # event publisher
        self.event_pub = rospy.Publisher("/robdos/stateEvents", StateEvent, queue_size=1)
        self.event_msg = StateEvent()

        # mavros velocity publisher
        self.mavros_vel_pub = rospy.Publisher("robdos/autopilot_commands", OverrideRCIn, queue_size=1)
        self.RCOR_msg = OverrideRCIn()

        # create subscriber for robot localization
        self.sub_localization = rospy.Subscriber('/robdos/simulatedOdometry', Odometry,
                                                 self.process_localization_message, queue_size=1)

        self.sub_waypoint = rospy.Subscriber('/robdos/mission/waypoints', Waypoint,
                                                  self.process_waypoint_message, queue_size=1)

        rospy.wait_for_service('waypoint_reached')
        self.waypointReached = rospy.ServiceProxy('waypoint_reached', Empty)

    # update list of waypoints
    def process_waypoint_message(self, waypoint_msg):
        if waypoint_msg.is_current == True:
            self.is_target_ready = True
            self.current_target = [waypoint_msg.x_lat, waypoint_msg.y_long, waypoint_msg.z_alt]
        else:
            self.is_target_ready = False

    # update position and orientation of robot (odometry)
    def process_localization_message(self, odometry_msg):
        self.robot_position_x = odometry_msg.pose.pose.position.x
        self.robot_position_y = odometry_msg.pose.pose.position.y
        self.robot_position_z = odometry_msg.pose.pose.position.z

        quaternion = (
            odometry_msg.pose.pose.orientation.x,
            odometry_msg.pose.pose.orientation.y,
            odometry_msg.pose.pose.orientation.z,
            odometry_msg.pose.pose.orientation.w)

        (self.robot_roll, self.robot_pitch, self.robot_yaw) = euler_from_quaternion(quaternion)

        self.is_localization_ready = True
        self.update_controller()

    def update_controller(self):
        if not self.is_localization_ready or not self.is_target_ready:
            # stop motors
            self.RCOR_msg = OverrideRCIn()
            self.RCOR_msg.channels = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
            self.mavros_vel_pub.publish(self.RCOR_msg)
            return

        distance_x = self.current_target[0] - self.robot_position_x
        distance_y = self.current_target[1] - self.robot_position_y
        distance_z = self.current_target[2] - self.robot_position_z

        desired_yaw = math.atan2(distance_y, distance_x)

        if desired_yaw >= self.robot_yaw:
            error_orientation = desired_yaw - self.robot_yaw
        else:
            error_orientation = self.robot_yaw - desired_yaw

        error_orientation = (error_orientation % 2 * math.pi) * self.rad2degrees

        error_position = math.sqrt(math.pow(distance_x, 2.0) + math.pow(distance_y, 2.0))

        error_depth = distance_z

        self.position_pid.SetPoint = 0.0
        self.position_pid.update(error_position)

        self.orientation_pid.SetPoint = desired_yaw
        self.orientation_pid.update(self.robot_yaw)
        self.orientation_pid.output = self.bound_limit(self.orientation_pid.output, -1000, 1000)

        self.depth_pid.SetPoint = self.current_target[2]
        self.depth_pid.update(self.robot_position_z)

        if abs(error_orientation) >= self.threshold_orientation or \
            error_position >= self.threshold_position or \
            error_depth >= self.threshold_depth:

            K_ORI = 1500 + self.orientation_pid.output
            K_ORI = self.bound_limit(K_ORI, 1100, 1900)

            K_POS = 1500 - self.position_pid.output
            K_POS = self.bound_limit(K_POS, 1100, 1900)

            K_DEP = 1500 - self.depth_pid.output
            K_DEP = self.bound_limit(K_DEP, 1100, 1900)

            # set speed thrusters
            self.RCOR_msg = OverrideRCIn()
            self.RCOR_msg.channels = [1500, 1500, 1500, K_ORI, K_POS, K_DEP, 1500, 1500]
            self.mavros_vel_pub.publish(self.RCOR_msg)

        else:
            self.waypointReached()

    def bound_limit(self, n, minn, maxn):
        return max(min(maxn, n), minn)

    def reconfig_callback(self, config, level):

        #ORIENTATION
        self.O_kp = config['ori_kp']
        self.O_kd = config['ori_kd']
        self.O_ki = config['ori_ki']
        self.O_windup = config['ori_wu']

        self.threshold_orientation = config['thr_orientation']

        self.orientation_pid = PID()
        self.orientation_pid.setSampleTime(0.01)
        self.orientation_pid.setKp(self.O_kp)
        self.orientation_pid.setKd(self.O_kd)
        self.orientation_pid.setKi(self.O_ki)
        self.orientation_pid.setWindup(self.O_windup)
        self.orientation_pid.error = 0.0
        self.orientation_pid.output = 0.0

        #POSITION
        self.P_kp = config['pos_kp']
        self.P_kd = config['pos_kd']
        self.P_ki = config['pos_ki']
        self.P_windup = config['pos_wu']

        self.threshold_position = config['thr_position']

        self.position_pid = PID()
        self.position_pid.setSampleTime(0.01)
        self.position_pid.setKp(self.P_kp)
        self.position_pid.setKd(self.P_kd)
        self.position_pid.setKi(self.P_ki)
        self.position_pid.setWindup(self.P_windup)
        self.position_pid.error = 0.0
        self.position_pid.output = 0.0

        #DEPTH
        self.D_kp = config['dep_kp']
        self.D_kd = config['dep_kd']
        self.D_ki = config['dep_ki']
        self.D_windup = config['dep_wu']

        self.threshold_depth = config['thr_depth']

        self.depth_pid = PID()
        self.depth_pid.setSampleTime(0.01)
        self.depth_pid.setKp(self.D_kp)
        self.depth_pid.setKd(self.D_kd)
        self.depth_pid.setKi(self.D_ki)
        self.depth_pid.setWindup(self.D_windup)
        self.depth_pid.error = 0.0
        self.depth_pid.output = 0.0

        self.limit_min = config['limit_min_thrusters']
        self.limit_max = config['limit_max_thrusters']

        return config


def main(args):
    vn = RobdosController()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down Robdos Controller Node."


if __name__ == '__main__':
    main(sys.argv)
