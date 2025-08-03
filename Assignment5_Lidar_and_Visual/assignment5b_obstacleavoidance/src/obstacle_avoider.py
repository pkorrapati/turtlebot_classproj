#!/usr/bin/env python3

import rospy
import numpy as np
from math import pi, radians
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
class TurtleBot:
    tuning_parameter = 0.3
    steering_speed = 0.3
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
        self.time_one=rospy.Time.now().to_sec()
    def lidar_remap(self,index):#This model makes most sense from 0-180 degrees from the "negative x-axis"
        if index<90:
            output=270+index
        else:
            output = index-90
        return output
    def controller(self,lidarData):
        print("In Controller")
        #for x in range(180):
            #index = self.lidar_remap(x)
        print( lidarData.ranges[0])
        for x in range(60,120):
            index=self.lidar_remap(x)
            if lidarData.ranges[index] < self.bubble_boundry:
                self.rebound(lidarData)
                break

    def rebound(self,lidarData):
        print("In rebound")
        numerator_array=[]
        denominator_array =[]
        for x in range(1,180):
            index=self.lidar_remap(x)
            numerator_array.append(x*lidarData.ranges[index])
            denominator_array.append(lidarData.ranges[index])
            print(numerator_array)
            print("den:")
            print(denominator_array)
        numerator_array=np.clip(numerator_array,0,3)
        denominator_array=np.clip(denominator_array,0,3)
        print(numerator_array)
        print("den:")
        print(denominator_array)
        numerator=sum(numerator_array)
        denominator=sum(denominator_array)
        rebound_angle=numerator/denominator
        print("Rebound Angle")
        print(rebound_angle)
        self.stop()
        self.turn()
        self.rotate(rebound_angle)
        self.stop()
    def turn(self):
        self.move_msg.angular.x=0.0
        self.move_msg.angular.y=0.0
        self.move_msg.angular.z= self.steering_speed
        self.move_msg.linear.x=0.0
        self.move_msg.linear.y=0.0
        self.move_msg.linear.z=0.0
        self.turtle_bot_move.publish(self.move_msg)
    def rotate(self,desired_angle):
        print("In rotate")       
        current_angle=0
        t0=rospy.Time.now().to_sec()
        while current_angle<round(radians(desired_angle),5):
            print("Desired")
            print(radians(desired_angle))
            print("Current:")
            print(current_angle)
            t1=rospy.Time.now().to_sec()
            current_angle = round(self.steering_speed * (t1-t0),5)
    def drive(self,lidarData):
        print("In Drive method")
        self.time_two=self.time_one
        self.time_one=rospy.Time.now().to_sec()
        self.delta_t=self.time_one-self.time_two
        self.bubble_boundry=0.4#self.tuning_parameter*##Velocity##*self.delta_t
        self.controller(lidarData)
        self.forward()
        # print("Past Percieve")
        # print("Past steer")
        # print(self.move_msg)
        #self.turtle_bot_move.publish(self.move_msg)
    def forward(self):
        print("In forwards")

        #Twist message for linear velocity components so turtlebot only drives forward
        self.move_msg.linear.x=0.15
        self.move_msg.linear.y=0.0
        self.move_msg.linear.z=0.0
        self.move_msg.angular.x=0.0
        self.move_msg.angular.y=0.0
        self.move_msg.angular.z=0.0
        self.turtle_bot_move.publish(self.move_msg)
    def stop(self):
        self.move_msg.angular.x=0.0
        self.move_msg.angular.y=0.0
        self.move_msg.angular.z= 0
        self.move_msg.linear.x=0.0
        self.move_msg.linear.y=0.0
        self.move_msg.linear.z=0.0
        self.turtle_bot_move.publish(self.move_msg)

class Segment:#Class to seperate different segments of LIDAR data
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