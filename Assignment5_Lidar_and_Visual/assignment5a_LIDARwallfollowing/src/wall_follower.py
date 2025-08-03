#!/usr/bin/env python3

import rospy
import numpy as np
from math import pi, radians
LOOKAHEAD=330
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class TurtleBot:
    errorSignal_2 = 0
    errorSignal_1 = 0
    errorSignal = 0
    def __init__(self):#Initialization script
        rospy.init_node('wall_follower',anonymous=True)#Starts node called 'wall_follower'
        print("Created Node")
        self.lidar_data = rospy.Subscriber("/scan", LaserScan,callback=self.drive)#Subscriber object that listens for LaserScan type messages from "/scan". It will then use the percieve method to process the message
        print("Created Subscriber")
        self.turtle_bot_move = rospy.Publisher("/cmd_vel",Twist,queue_size=10)#Publisher object that publishes a Twist type message to "/cmd_vel" and has a queue buffer with size of 10
        print("Created Publisher")
        self.move_msg=Twist()
    def percieve(self, lidarData):
        #self.left_wall = Wall(85,95,samples=5,data=lidarData)
        self.right_wall = lidarData.ranges[LOOKAHEAD]#Wall(330,350,samples=5,data=lidarData)
        print("Right Wall")
        print(self.right_wall)
    def controller(self):
        TARGET=.7
        ## PID GAINS ##
        P_GAIN = 5
        I_GAIN = 2
        D_GAIN = 2
        K_ONE = P_GAIN + I_GAIN + D_GAIN#Gains for discrete PID
        K_TWO = -P_GAIN + -2 * D_GAIN
        K_THREE = D_GAIN
        ## Error Signals##
        self.errorSignal_2 = self.errorSignal_1#Errors for discrete PID
        self.errorSignal_1 = self.errorSignal
        self.errorSignal = TARGET - self.right_wall
        ##Output Signal##
        output=self.errorSignal * K_ONE + self.errorSignal_1 * K_TWO + self.errorSignal_2 * K_THREE#output signal for discrete PID
        print("error:")
        print(self.errorSignal)
        # print("error1:")
        # print(self.errorSignal_1)
        # print("error2:")
        # print(self.errorSignal_2)
        print("output:")
        print(output)
        return output
    def forward(self):
        #Twist message for linear velocity components so turtlebot only drives forward
        self.move_msg.linear.x=0.15
        self.move_msg.linear.y=0.0
        self.move_msg.linear.z=0.0
        # print(self.move_msg)
    def backward(self):
        #Twist message for linear velocity components so turtlebot only drives forward
        self.move_msg.linear.x=-0.05
        self.move_msg.linear.y=0.0
        self.move_msg.linear.z=0.0
        # print(self.move_msg)
    def steer(self):
        self.move_msg.angular.x=0.0
        self.move_msg.angular.y=0.0
        self.move_msg.angular.z= self.controller()
        # print(self.move_msg)
    def drive(self,lidarData):
        # print("In Drive method")
        self.percieve(lidarData)
        # print("Past Percieve")
        print("LIDAR Front:")
        #print(lidarData.ranges[0])
        #print(lidarData.ranges[LOOKAHEAD])
        if lidarData.ranges[0]<.15:
            self.stop()
            self.backward()
        else:
            self.forward()
            self.steer()
        # print("Past Forward")
        #self.steer()
        # print("Past steer")
        # print(self.move_msg)
        self.turtle_bot_move.publish(self.move_msg)
    def stop(self):
        self.move_msg.angular.x=0.0
        self.move_msg.angular.y=0.0
        self.move_msg.angular.z= 0
        self.move_msg.linear.x=0.0
        self.move_msg.linear.y=0.0
        self.move_msg.linear.z=0.0
        self.turtle_bot_move.publish(self.move_msg)

class Wall:
    def __init__(self,minAngle,maxAngle,samples,data):
        self.minAngle = minAngle#Defines the starting angle of a sweep
        self.maxAngle = maxAngle#Defines the end angle of a sweep for walls
        self.samples = samples#Defines how many data points to capture in each sweep
        self.datapoints = []#Creates empty list to store datapoints
        iterator=((self.maxAngle-self.minAngle)/self.samples)#Determines size of iterator to go through entire angle sweep
        i=0
        while i<samples:
            self.datapoints.append(data.ranges[int(self.minAngle+iterator*i)])#Appends range data to end of list
            i=i+1
            #print(i)
        
        self.datapoints = np.clip(self.datapoints,0,3)
        print(self.datapoints)
        self.average = np.mean(self.datapoints)#Calculates mean of datapoints
        self.variance = np.var(self.datapoints)#Calculates variance of datapoints



if __name__== '__main__':
    try:
        tb=TurtleBot()
        print("trying")
        tb.stop()
        rospy.spin()
    except rospy.ROSInterruptException:
        tb_stop=TurtleBot()
        tb_stop.stop()