#!/usr/bin/python

from VexBotDriveController import *
import Tkinter
from Tkinter import *
from Tkconstants import *

import roslib; roslib.load_manifest('VexBot')
import rospy

from geometry_msgs.msg import Twist


class Visualizer(Tkinter.Frame):
    def __init__(self, real):
        rospy.init_node('Visualizer')
        Tkinter.Frame.__init__(self, root)
        self.controller = VexBotTankDrive()
        rospy.Subscriber("cmd_vel", Twist, self.callback)
        self.Left = 0.0
        self.Right = 0.0
        self.headLabel = Label(self, justify=CENTER, font=("Helvetica", 18))
        self.headLabel["text"] = "Motor Status"
        self.headLabel.grid(row=0, columnspan=2)
        self.line1 = Label(self, text="--------------------------", font=("Helvetica", 14))
        self.line1.grid(row=1, columnspan=2)
        #Draw labels
        self.LFlabel = Label(self, text="Left", font=("Helvetica", 12))
        self.LFlabel.grid(row=2, column=0)
        self.RFlabel = Label(self, text="Right", font=("Helvetica", 12))
        self.RFlabel.grid(row=2, column=1)
        self.line2 = Label(self, text="--------------------------", font=("Helvetica", 14))
        self.line2.grid(row=4, columnspan=2)
        #Get color values
        Lcolor = self.colorize(self.Left)
        Rcolor = self.colorize(self.Right)
        #Get command values
        LV = self.valuize(self.Left)
        RV = self.valuize(self.Right)
        #Draw values
        self.LFvalue = Label(self, text=LV, bg=Lcolor, fg="white", font=("Courier", 18))
        self.LFvalue.grid(row=3, column=0)
        self.RFvalue = Label(self, text=RV, bg=Rcolor, fg="white", font=("Courier", 18))
        self.RFvalue.grid(row=3, column=1)
        self.KEY = Label(self, text="Values range from 127 to -127", font=("Helvetica", 12))
        self.KEY.grid(row=5, columnspan=2)

    def valuize(self, value):
        """Convert speed value into string value"""
        raw_value = int(127 * value)
        if (raw_value <= 127 and raw_value >= 100):
            return "+" + str(raw_value)
        elif (raw_value <= 99 and raw_value >= 10):
            return "+0" + str(raw_value)
        elif (raw_value <= 9 and raw_value > 0):
            return "+00" + str(raw_value)
        elif (raw_value == 0):
            return "0000"
        elif (raw_value < 0 and raw_value >= -9):
            return "-00" + str(abs(raw_value))
        elif (raw_value < 9 and raw_value >= -99):
            return "-0" + str(abs(raw_value))
        elif (raw_value < -99 and raw_value >= -127):
            return "-" + str(abs(raw_value))
            

    def colorize(self, value):
        """Convert speed value into color"""
        blue = 0
        green = 0
        red = 0
        if (value == 0.0 or value == -0.0):
            green = 0
            red = 0
        elif (value > 0.0):
            red = 0
            green = int(255 * value)
        elif (value < -0.0):
            green = 0
            red = int(255 * abs(value))
        else:
            green = 0
            red = 0
            blue = 255
        tk_rgb = "#%02x%02x%02x" % (red, green, blue)
        return tk_rgb
            

    def callback(self, data):
        """Send Twist commands to the robot"""
        X = data.linear.x
        Z = data.angular.z
        #print "X : " + str(X) + " Y : " + str(Y) + " Z : " + str(Z)
        commands = self.controller.Compute(X, Z)
        self.Left = commands[0]
        self.Right = commands[1]
        #Get color values
        Lcolor = self.colorize(self.Left)
        Rcolor = self.colorize(self.Right)
        #Get command values
        LV = self.valuize(self.Left)
        RV = self.valuize(self.Right)
        #Draw values
        self.LFvalue = Label(self, text=LV, bg=Lcolor, fg="white", font=("Courier", 18))
        self.LFvalue.grid(row=3, column=0)
        self.RFvalue = Label(self, text=RV, bg=Rcolor, fg="white", font=("Courier", 18))
        self.RFvalue.grid(row=3, column=1)


if __name__ == "__main__":
    root = Tkinter.Tk()
    root.title("VexBot Drive Visualizer")
    Visualizer(root).pack()
    root.mainloop()
