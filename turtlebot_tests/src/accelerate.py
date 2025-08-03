#!/usr/bin/env python3

import rospy
import numpy as np

from math import pi, radians

from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry

import tf2_ros
import matplotlib.pyplot as plt

def limVal(val, lim):
    return val if val < lim else lim

class TurtleBot:

    def __init__(self):
        # Creates a node 'turtlebot_circle_driver'. Using anonymous=True makes it unique.
        rospy.init_node('turtlebot_square_driver', anonymous=True)

        # Reset turtlesim by calling the reset service
        # Not necessary when running from for launch files
        rospy.wait_for_service('/gazebo/reset_world')
        resetTurtle = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        resetTurtle()

        # Publisher which will publish to the topic '/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.update_pose)
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_subber = tf2_ros.TransformListener(self.tfBuffer)

        self.rate = rospy.Rate(30)   
        self.isAlive = False
        self.vel = 0   
        
    def update_pose(self, data):
        if not self.isAlive:
            self.isAlive = True
    
    def traverseLine(self):
        """Accelerate"""
        
        # Get the input from the user.        
        # Acceleration
        a = rospy.get_param('~a')
        # Target velocity
        v = rospy.get_param('~v')

        vel_msg = Twist()

        simTime = 4

        X =[]
        Y =[]
        T =[]
        Vt = []

        while not self.isAlive:
            pass
        
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        # Angular velocity in the z-axis.
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        # Publishing our vel_msg
        self.velocity_publisher.publish(vel_msg)
        self.rate.sleep()

        t_start = rospy.get_time()
        t = t_start

        while t <  t_start + simTime:                                                
            self.vel = limVal(a * (t - t_start), v)
            
            try:                          
                trans = self.tfBuffer.lookup_transform('odom', 'base_footprint', rospy.Time(t)) #base_footprint
                T.extend([t])
                X.extend([trans.transform.translation.x])
                Y.extend([trans.transform.translation.y])                                
                Vt.extend([self.vel])
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue
                       
            vel_msg.linear.x = self.vel
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0      

            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0
            
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

            t = rospy.get_time()
            
        # Stop motion
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        
        Tpl = np.subtract(T, t_start)

        data = np.column_stack((Tpl, X))
        data = np.unique(data, axis=0)

        dX = data[:,1]
        dX[0:-1] = np.divide(data[1:,1] - data[0:-1,1], data[1:,0] - data[0:-1,0])
        dX[-1] = dX[-2]

        data = np.column_stack((data, dX))
                
        
        plt.plot(Tpl,Vt)
        plt.plot(data[:,0],data[:,2])        
        plt.title('X vs t location of base_footprint')
        plt.xlabel('time')
        # plt.axis('equal')
        # plt.legend(['X-Position', 'X-Velocity [Expected]', 'X-Velocity [Actual]'])
        plt.legend(['X-Velocity [Expected]', 'X-Velocity [Actual]'])
        plt.show()

        # If we press control + C, the node will stop.
        # rospy.spin()

if __name__ == '__main__':
    try:
        x = TurtleBot()
        x.traverseLine()

    except rospy.ROSInterruptException:
        pass
