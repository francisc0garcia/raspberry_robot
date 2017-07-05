#!/usr/bin/env python

import math
import rospy

# import dependencies
from RobotController import *
from mavros_msgs.msg import WaypointList, Waypoint
from PID import *

class Robot2Marker:
    def __init__(self):
        # Init ros node
        rospy.init_node('Robot2Marker')

        self.degrees2rad = math.pi / 180.0
        self.rad2degrees = 180.0 / math.pi

        self.robot = RobotController()

        self.robot.marker_id_target = 0
        self.robot.marker_id_robot = 3

        self.current_target = None
        self.is_target_ready = False

        [self.threshold_position, self.threshold_orientation] = [0.2, 5]
        # [self.P_kp, self.P_kd, self.P_ki, self.P_windup] = [0.2, 0.0, 0.0, 0.0]
        # [self.O_kp, self.O_kd, self.O_ki, self.O_windup] = [0.6, 0.0, 0.0, 0.0]
        [self.P_kp, self.P_kd, self.P_ki, self.P_windup] = [0.18, 0.0, 0.0, 0.0]
        [self.O_kp, self.O_kd, self.O_ki, self.O_windup] = [0.5, 0.0, 0.0, 0.0]
        self.freq = 50.0

        # ORIENTATION
        self.orientation_pid = PID()
        self.orientation_pid.setSampleTime(0.1)
        self.orientation_pid.setKp(self.O_kp)
        self.orientation_pid.setKd(self.O_kd)
        self.orientation_pid.setKi(self.O_ki)
        self.orientation_pid.setWindup(self.O_windup)
        self.orientation_pid.error = 0.0
        self.orientation_pid.output = 0.0

        # POSITION
        self.position_pid = PID()
        self.position_pid.setSampleTime(0.1)
        self.position_pid.setKp(self.P_kp)
        self.position_pid.setKd(self.P_kd)
        self.position_pid.setKi(self.P_ki)
        self.position_pid.setWindup(self.P_windup)
        self.position_pid.error = 0.0
        self.position_pid.output = 0.0

        self.rate = rospy.Rate(self.freq)

        self.sub_waypoint = rospy.Subscriber('/mavros/mission/waypoints', Waypoint,
                                             self.process_waypoint_message, queue_size=1)

        # infinity loop
        while not rospy.is_shutdown():
            # [robot_position_x, robot_position_y, robot_orientation] = self.robot.get_position()
            # [target_x, target_y, target_orientation] = self.robot.get_target_position()

            # rospy.loginfo("robot position:" + str([robot_position_x, robot_position_y, robot_orientation]))
            if self.robot.new_message is True:
                self.update_controller()
                self.robot.new_message = False

            self.rate.sleep()

    def update_controller(self):
        if not self.is_target_ready:
            # stop motors
            self.robot.move(0.0)
            return
        [target_x, target_y, target_orientation] = self.robot.get_target_position()
        [robot_position_x, robot_position_y, robot_orientation] = self.robot.get_position()
        # distance_x = self.current_target[0] - robot_position_x
        # distance_y = self.current_target[1] - robot_position_y

        distance_x = target_x - robot_position_x
        distance_y = target_y - robot_position_y

        desired_yaw = math.atan2(distance_y, distance_x)

        # rospy.loginfo("x %f y %f o %f ", robot_position_x, robot_position_y, robot_orientation)
        # rospy.loginfo("tx %f ty %f to %f ", self.current_target[0], self.current_target[1], desired_yaw)

        # robot_orientation = robot_orientation + math.pi/2.0

        error_orientation = desired_yaw - robot_orientation

        error_position = math.sqrt(math.pow(distance_x, 2.0) + math.pow(distance_y, 2.0))

        self.position_pid.SetPoint = 0.0
        self.position_pid.update(error_position)
        self.position_pid.output = self.bound_limit(self.position_pid.output, -10.0, 10.0)

        self.orientation_pid.SetPoint = desired_yaw
        self.orientation_pid.update(robot_orientation)
        self.orientation_pid.output = self.bound_limit(self.orientation_pid.output, -10, 10)

        if abs(error_orientation) >= self.threshold_orientation or \
           abs(error_position) >= self.threshold_position:
            self.robot.move_rotate(-self.position_pid.output, -self.orientation_pid.output)
            # self.robot.move_rotate(0.0, -self.orientation_pid.output)
        else:
            self.robot.move_rotate(0.0, 0.0)

    def bound_limit(self, n, minn, maxn):
        return max(min(maxn, n), minn)

    # update list of waypoints
    def process_waypoint_message(self, waypoint_msg):
        if waypoint_msg.is_current is True:
            self.is_target_ready = True
            self.current_target = [waypoint_msg.x_lat, waypoint_msg.y_long, waypoint_msg.z_alt]
        else:
            self.is_target_ready = False


def main(args):
    try:
        ic = Robot2Marker()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)