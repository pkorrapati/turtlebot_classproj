#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty
from math import pow, atan2, sqrt, pi, cos, sin, radians
import numpy as np

# Remaps (-pi, pi) to (0, 2pi)
def remapAngle(angle):
    return round((angle + (2*pi)) % (2*pi), 4)

def getRotationMatrix(angle):
    return np.array([[cos(angle), -sin(angle)],
                  [sin(angle),  cos(angle)]])

class TurtleBot:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('turtlebot_square_driver', anonymous=True)

        #reset turtlesim by calling the reset service
        rospy.wait_for_service('reset')
        resetTurtle = rospy.ServiceProxy('reset', Empty)
        resetTurtle()

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',
                                                  Twist, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose',
                                                Pose, self.update_pose)

        self.pose = Pose()
        self.rate = rospy.Rate(50)
        self.blank = True
        self.stage = 1
        self.inMotion = True

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

        if self.blank:            
            self.startX = self.pose.x
            self.startY = self.pose.y
            self.blank = False

        if self.stage == 1 and (self.pose.x - self.startX) >= 2:
            self.stage += 1
        
        elif self.stage == 2 and (self.pose.y - self.startY) >= 2:
            self.stage += 1
        
        elif self.stage == 3 and (self.pose.x - self.startX) <= 0:
            self.stage += 1
        
        elif self.stage == 4 and (self.pose.y - self.startY) <= 0:
            self.stage += 1
            self.inMotion = False


    def traverseSquare(self):
        """Move in a square."""
        
        # Get the input from the user.        
        v = rospy.get_param('~v')
        w = rospy.get_param('~w')

        vel_msg = Twist()

        while self.blank:
            pass
       
        # while self.pose.orientation:
        # for i in range(100):
        while self.inMotion:
            rospy.loginfo(self.pose.x)
            # Velocity in Inertial Frame
            dX_i = np.array([0,0]).T

            if self.stage == 1:
                dX_i = np.array([v, 0]).T #move right
            
            elif self.stage == 2:                
                dX_i = np.array([0, v]).T #move up
            
            elif self.stage == 3:                
                dX_i = np.array([-v,0]).T #move left
            elif self.stage == 4:                
                dX_i = np.array([0,-v]).T #move down
            
            # Velocity in Robot Frame
            dX_b = np.dot(getRotationMatrix(remapAngle(self.pose.theta)).T, dX_i)

            # Linear velocity in the x-axis.
            vel_msg.linear.x = dX_b[0]
            vel_msg.linear.y = dX_b[1]
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = w

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        # Stop motion
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        # If we press control + C, the node will stop.
        # rospy.spin()

if __name__ == '__main__':
    try:
        x = TurtleBot()
        x.traverseSquare()

    except rospy.ROSInterruptException:
        pass
