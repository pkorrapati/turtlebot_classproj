#!/usr/bin/env python3

import numpy as np
import cv2

import rospy

# import imagezmq
# import argparse
# import time
# import os

from math import pi, radians, degrees

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import matplotlib.pyplot as plt
plt.ion()

counter = 0

# Set this to false to skip matplotlib 
plotThings = True

def limit(value, limitL, limitU):
    if value > limitU:
        return limitU
    elif value < limitL:
        return limitL
    
    return value

class VisualCortex:
    def __init__(self):                
        rospy.init_node('visual_cortex', anonymous=True)

        self.rate = rospy.Rate(100)

        # Subscribers        
        self.sub_lidar = rospy.Subscriber('/scan', LaserScan, self.echo)

        # Publisher
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.isAlive = False    
        self.newEcho = False

        self.angles = []
        self.vel_msg = Twist()

        self.vel_msg.linear.x = 0
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0

        if plotThings:
            
            self.fig = plt.figure()
            self.ax = self.fig.add_subplot(121)
            self.bx = self.fig.add_subplot(122, projection='polar')
            self.bx.set_theta_zero_location("N")
    
    def echo(self, data):
        global counter

        self.minAngle = -radians(90)
        self.maxAngle = radians(90)

        self.minScanAngle = data.angle_min
        self.maxScanAngle = data.angle_max
        self.incAngle = data.angle_increment
        
        count = len(data.ranges)           

        # Indices of Left and Right sides of the robot
        self.lIndx = round(self.maxAngle/self.incAngle)
        self.rIndx = round(radians(270)/self.incAngle) - count

        # Count of Subset
        subCount = self.lIndx - self.rIndx

        if len(self.angles) != count:
            # self.angles = np.linspace(self.minAngle, self.maxAngle, num=count)
            self.angles = np.linspace(self.minAngle, self.maxAngle, num=subCount)
        
        self.scan_ranges = np.hstack((data.ranges[self.rIndx:], data.ranges[:self.lIndx]))
        
        # zero index
        self.zIndx = -self.rIndx -1
        
        # Clean up inf in data
        infIndx = np.where(self.scan_ranges == np.inf)
        self.firstInf = subCount

        if len(infIndx) > 0 and len(infIndx[0]) > 0:
            self.firstInf = infIndx[0][0]
            # print(firstInf)

        # self.scan_ranges[np.where(self.scan_ranges == np.inf)] = 4.0
        self.scan_ranges[infIndx] = 4.0
        
        # self.scan_intensities = data.intensities        
        
        if not self.isAlive:
            if plotThings:             
                self.distaPlot, = self.ax.plot([self.minAngle, self.maxAngle], [0,4])
                self.distbPlot, = self.bx.plot([0, 2*pi], [0,4])

            self.isAlive = True
        
        counter = (counter + 1) % 5
        self.newEcho = True

    def showData(self):
        while not rospy.is_shutdown():        
            if self.isAlive:

                # Plotting
                if counter == 0 and plotThings:
                    self.distaPlot.set_xdata(self.angles[:self.firstInf])
                    self.distaPlot.set_ydata(self.scan_ranges[:self.firstInf])   

                    self.distbPlot.set_xdata(self.angles[:self.firstInf])
                    self.distbPlot.set_ydata(self.scan_ranges[:self.firstInf])   

                    self.fig.canvas.draw()
                    self.fig.canvas.flush_events()
                    plt.subplots_adjust(bottom=0.1, top=0.9)

                if self.scan_ranges[self.zIndx] > 0.5:
                    self.vel_msg.linear.x = 0.1
                else:
                    self.vel_msg.linear.x = 0

                # self.vel_msg.angular.z = 0

                self.velocity_publisher.publish(self.vel_msg)

                self.rate.sleep()  

if __name__ == '__main__':
    try:
        vNode = VisualCortex()
        vNode.showData()
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass
