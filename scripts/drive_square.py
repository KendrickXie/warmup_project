#!/usr/bin/env python3

import rospy

# msgs needed for /cmd_vel
from geometry_msgs.msg import Twist, Vector3

class DriveSquare(object):
    """ This node publishes ROS messages driving the robot in a square shape """

    def __init__(self):
        # initialize the ROS node
        rospy.init_node('drive_square')
        # setup publisher to the cmd_vel ROS topic
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # create a default twist msg with values of all zeros
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin,angular=ang)

    def drive_straight(self):
        # set Twist message to move forward and publish it
        self.twist.linear.x = 0.2
        self.twist_pub.publish(self.twist)
        
        # set Twist message to stop and publish it after waiting 2 seconds
        self.twist.linear.x = 0.0
        rospy.sleep(2)
        self.twist_pub.publish(self.twist)

    def turn_90_degrees(self):
        # set Twist message to turn and publish it
        self.twist.angular.z = 0.6
        self.twist_pub.publish(self.twist)
        
        # set Twist message to stop and publish it after waiting 2.75 seconds
        self.twist.angular.z = 0.0
        rospy.sleep(2.75)
        self.twist_pub.publish(self.twist)

    def run(self):
        # allow the publisher enough time to set up before publishing the first msg
        rospy.sleep(1)
        # call the drive_straight() and turn_90_degrees() methods 4 times to drive in a square
        for i in range(4):
            self.drive_straight()
            self.turn_90_degrees()
            
if __name__ == '__main__':
    # declare the ROS node and run it
    node = DriveSquare()
    node.run()
