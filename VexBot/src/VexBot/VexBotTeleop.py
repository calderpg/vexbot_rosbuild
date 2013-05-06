#!/usr/bin/env python

#   Calder Phillips-Grafflin
#
#   Generic 2DOF vehicle teleoperation controller using joystick input.
#

import sys
import roslib; roslib.load_manifest('VexBot')
import rospy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from ControllerMappings import *


class Teleop:
    def __init__(self, joystick_topic, cmd_topic, max_linear, max_angular, pub_rate):
        self.mapping = XboxMapping()
        self.last_joy = None
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.cmd_pub = rospy.Publisher(cmd_topic, Twist)
        self.joy_sub = rospy.Subscriber(joystick_topic, Joy, self.joy_cb)
        rospy.loginfo("Basic Xlinear+Zangular teleop node running...")
        rate = rospy.Rate(rospy.get_param('~hz', pub_rate))
        while not rospy.is_shutdown():
            rate.sleep()
            cleaned = self.convert(self.last_joy)
            self.cmd_pub.publish(cleaned)

    def merge_triggers(self, left_trigger, right_trigger):
        left = (-left_trigger - 1.0) / 2.0
        right = (right_trigger + 1.0) / 2.0
        merged = left + right
        #print "Raw: " + str(left) + " [L], " + str(right) + " [R], Merged: " + str(merged)
        return merged

    def convert(self, joy_msg):
        cmd = Twist()
        if (joy_msg == None):
            return cmd
        msg_dict = self.mapping.msg_to_dict(joy_msg)
        speed = msg_dict["left_stick_UDaxis"]
        ltrig = msg_dict["left_trigger"]
        rtrig = msg_dict["right_trigger"]
        angle = self.merge_triggers(ltrig, rtrig)
        linear_velocity = self.max_linear * speed
        angular_velocity = self.max_angular * angle
        deadman = msg_dict["A"]
        if (deadman == 1):
            cmd.linear.x = linear_velocity
            cmd.angular.z = angular_velocity
        return cmd

    def joy_cb(self, msg):
        self.last_joy = msg


if __name__ == "__main__":
    rospy.init_node("vexbot_teleop")
    joy_topic = rospy.get_param("~joy_topic", "joy")
    cmd_topic = rospy.get_param("~cmd_topic", "cmd_vel")
    max_linear = rospy.get_param("~max_linear", 1.0)
    max_angular = rospy.get_param("~max_angular", 1.0)
    update_hz = rospy.get_param("~rate", 30.0)
    Teleop(joy_topic, cmd_topic, max_linear, max_angular, update_hz)
