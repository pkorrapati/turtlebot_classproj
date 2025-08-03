#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty
from math import pow, atan2, sqrt, pi, radians, degrees

# Remaps (-pi, pi) to (0, 2pi)
def remapAngle(angle):
    return round((angle + (2*pi)) % (2*pi), 4)

class TurtleBot:

    def __init__(self):
        # Creates a node with name 'turtlebot_circle_driver' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('turtlebot_circle_driver', anonymous=True)

        #reset turtlesim by calling the reset service
        rospy.wait_for_service('reset')
        resetTurtle = rospy.ServiceProxy('reset', Empty)
        resetTurtle()

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',
                                                  Twist, queue_size=3)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose',
                                                Pose, self.update_pose)

        self.pose = Pose()
        self.rate = rospy.Rate(10)
        self.blank = True

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

        remappedAngle = remapAngle(self.pose.theta)

        if self.blank:
            self.rotations = 0            
            self.lastOrientation = remappedAngle
            self.blank = False
        
        if remappedAngle >= radians(0) and self.lastOrientation > radians(357):
            self.rotations += 1 
        
        self.lastOrientation = remappedAngle

    def traverseCircle(self):
        """Move in a circle."""
        
        # Get the input from the user.        
        r = rospy.get_param('~r')
        w = rospy.get_param('~w')

        vel_msg = Twist()

        while self.blank:
            pass

        while ((self.rotations*2*pi) + remapAngle(self.pose.theta)) < 2*pi:
            rospy.loginfo((self.rotations*2*pi) + remapAngle(self.pose.theta))
            
        # while True:
            # Linear velocity in the x-axis.
            vel_msg.linear.x = w * r
            vel_msg.linear.y = 0
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
        x.traverseCircle()
    except rospy.ROSInterruptException:
        pass
