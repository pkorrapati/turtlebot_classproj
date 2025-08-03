#!/usr/bin/env python3

# Go straight till obstacle < 0.5 m in the front, turn left until obstacle is > 0.5

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

def extractRanges(ranges, center, Mins, Plus):
    if (center - Mins) < 0 and (center + Plus) > 0 :
        return np.hstack((ranges[(center - Mins):], ranges[:(center + Plus)]))
    else:
        return np.array(ranges[(center - Mins):(center + Plus)])

def cylToCart(angles, dists):
    X = np.multiply(dists, np.cos(angles))
    Y = np.multiply(dists, np.sin(angles))

    return X, Y

def fitLine(X, Y):
    A = np.vstack([X, np.ones(len(X))]).T

    m,c = np.linalg.lstsq(A, Y, rcond=0)[0]
    return [1, -m, -c]

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
        
        self.vel_msg = Twist()

        self.dist_ref = 0.7
        self.wall_dist = 0

        self.fAngle = radians(0)
        self.rAngle = self.fAngle - radians(90)
        self.lAngle = self.fAngle + radians(90)

        # Range of right side angles
        self.rPlus = radians(25)
        self.rMins = radians(15)

        # Range of front angles
        self.fPlus = radians(15)
        self.fMins = radians(15)

        self.angles = []

        if plotThings:            
            self.fig = plt.figure()
            self.ax = self.fig.add_subplot(121)
            self.bx = self.fig.add_subplot(122, projection='polar')
            self.bx.set_theta_zero_location("N")
    
    def echo(self, data):
        global counter

        # self.minScanAngle = data.angle_min
        # self.maxScanAngle = data.angle_max
        self.incAngle = data.angle_increment
        # self.scan_intensities = data.intensities    
        
        ranges = np.array(data.ranges)
        
        # Clean up inf in data
        ranges[np.where(ranges == np.inf)] = 10.0

        count = len(ranges)           

        # Indices of Left and Right sides of the robot
        # use these only with data.ranges
        fIndx = round(self.fAngle/self.incAngle) # Index of heading
        lIndx = round(self.lAngle/self.incAngle) # Index of left
        rIndx = round((2*pi + self.rAngle)/self.incAngle) - count # Index of right, in negative

        # Right region
        self.rPlusIndx = round(self.rPlus/self.incAngle)
        self.rMinsIndx = round(self.rMins/self.incAngle)

        # Front region
        self.fPlusIndx = round(self.fPlus/self.incAngle)
        self.fMinsIndx = round(self.fMins/self.incAngle)

        # Count of Subset -90 to 90 degree
        frontCount = lIndx - rIndx

        if len(self.angles) != count:            
            self.angles = np.linspace(self.rAngle, self.lAngle, num=frontCount)
        
        # full front
        self.scan_ranges = np.hstack((ranges[rIndx:], ranges[:lIndx]))
        
        # zero front index
        self.zIndx = -rIndx -1
        
        # right scan    
        self.scan_right = extractRanges(ranges, rIndx, self.rMinsIndx, self.rPlusIndx)
        self.angl_right = np.linspace(self.rAngle - self.rMins, self.rAngle + self.rPlus, num=len(self.scan_right))

        X, Y = cylToCart(self.angl_right, self.scan_right)

        self.wall_eq = fitLine(X, Y)
        
        self.wall_dist_prev = self.wall_dist
        self.wall_dist = np.dot([0,0,1], self.wall_eq)
        
        # print(self.wall_dist)
        
        # front scan    
        self.scan_front = extractRanges(ranges, fIndx, self.fMinsIndx, self.fPlusIndx)
        self.angl_front = np.linspace(self.fAngle - self.fMins, self.fAngle + self.fPlus, num=len(self.scan_front))

        # Clean up inf in data
        # infIndx = np.where(self.scan_ranges == np.inf)
        # self.firstInf = frontCount
        # self.lastInf = frontCount

        # if len(infIndx) > 0 and len(infIndx[0]) > 0:
        #     self.firstInf = infIndx[0][0]
        #     self.lastInf = infIndx[0][-1]
            # print(firstInf)

        # self.scan_ranges[np.where(self.scan_ranges == np.inf)] = 4.0
        # self.scan_ranges[infIndx] = 4.0
        
        if not self.isAlive:
            if plotThings:             
                self.distaPlot, = self.ax.plot([self.rAngle, self.lAngle], [0,4])
                self.distbPlot, = self.bx.plot([0, 2*pi], [0,4])

            self.isAlive = True
        
        counter = (counter + 1) % 5
        self.newEcho = True

    def showData(self):
        while not rospy.is_shutdown():        
            if self.isAlive:

                # Plotting
                if counter == 0 and plotThings:
                    # self.distaPlot.set_xdata(self.angles[:self.firstInf])
                    # self.distaPlot.set_ydata(self.scan_ranges[:self.firstInf])   

                    # self.distbPlot.set_xdata(self.angles[:self.firstInf])
                    # self.distbPlot.set_ydata(self.scan_ranges[:self.firstInf])   

                    self.distaPlot.set_xdata(self.angles)
                    self.distaPlot.set_ydata(self.scan_ranges)   

                    self.distbPlot.set_xdata(self.angles)
                    self.distbPlot.set_ydata(self.scan_ranges)   

                    self.fig.canvas.draw()
                    self.fig.canvas.flush_events()
                    plt.subplots_adjust(bottom=0.1, top=0.9)

                if self.scan_front.min() > 0.5:
                    self.vel_msg.linear.x = 0.15
                    # self.vel_msg.angular.z = limit(-0.0005*(1 / self.wall_eq[1]) +  0.2 * (self.wall_dist - 0.7), -0.2, 0.2)                                        
                    self.vel_msg.angular.z = limit(-0.1 * (self.wall_dist - 0.7) + 0.4* (self.wall_dist - self.wall_dist_prev) , -0.2, 0.2)
                    # self.vel_msg.angular.z = limit(0.2 * (self.wall_dist - self.wall_dist_prev) , -0.2, 0.2)
                    print(self.vel_msg.angular.z)
                else:
                    self.vel_msg.linear.x = 0
                    self.vel_msg.angular.z = 0.1

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
