#!/usr/bin/python

#   Calder Phillips-Grafflin
#
#   Driver for a 2DOF robot controlled by an Arduino (or compatible) microcontroller
#   over a serial interface. To implement a new hardware interface, replace the
#   Ardubot(port) command with the relevant call to your driver.

from ArduController import *
from VexBotDriveController import *
import time

import roslib; roslib.load_manifest('VexBot')
import rospy

from geometry_msgs.msg import Twist


class ArduBotDriver:
    def __init__(self, port, max_linear, max_angular, max_total):
        rospy.loginfo("Starting VexBot driver node with port: " + port)
        self.micro = ArduBot(port)
        self.controller = VexBotTankDrive(max_linear, max_angular, max_total)
        self.m0 = 0.0
        self.m1 = 0.0
        rospy.Subscriber("cmd_vel", Twist, self.callback)
        rate = rospy.Rate(rospy.get_param('~hz', 32))
        self.last_time = time.clock()
        while not rospy.is_shutdown():
            rate.sleep()
            new_time = time.clock()
            time_difference = new_time - self.last_time
            if time_difference >= 1.0:
                self.m0 = 0.0
                self.m1 = 0.0
            result = self.micro.GoPercent(self.m0, self.m1)
            #print result

    def callback(self, data):
        """Convert Twist commands to motor commands"""
        X = data.linear.x
        Z = data.angular.z
        commands = self.controller.Compute(X, Z)
        self.last_time = time.clock()
        self.m0 = commands[0]
        self.m1 = commands[1]


if __name__ == "__main__":
    rospy.init_node('vexbot_driver')
    port = rospy.get_param("~port", "/dev/ttyACM0")
    max_linear = rospy.get_param("~max_linear", 1.0)
    max_angular = rospy.get_param("~max_angular", 1.0)
    max_total = rospy.get_param("~max_total", 1.0)
    ArduBotDriver(port, max_linear, max_angular, max_total)
