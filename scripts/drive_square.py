#!/usr/bin/env python3

import rospy

# msgs needed for /cmd_vel
from geometry_msgs.msg import Twist, Vector3

class DriveSquare(object):
    """ This node publishes ROS messages containing the 3D coordinates of a single point """

    def __init__(self):
        # initialize the ROS node
        rospy.init_node('drive_square')
        # setup publisher to the cmd_vel ROS topic
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # create a default twist msg (all values 0).
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin,angular=ang)

    def drive_straight(self):
        #
        self.twist.linear.x = 0.2

        

        #
        self.twist_pub.publish(self.twist)
        
        #
        self.twist.linear.x = 0.0

        rospy.sleep(2)
        self.twist_pub.publish(self.twist)

    def turn_90_degrees(self):
        #
        self.twist.angular.z = 0.6

        #
        self.twist_pub.publish(self.twist)
        
        #
        self.twist.angular.z = 0.0

        rospy.sleep(2.75)
        self.twist_pub.publish(self.twist)

    def run(self):
        # allow the publisher enough time to set up before publishing the first msg
        rospy.sleep(1)
        # setup the Twist message we want to send
        for i in range(4):
            self.drive_straight()
            self.turn_90_degrees()
            
if __name__ == '__main__':
    # instantiate the ROS node and run it
    node = DriveSquare()
    node.run()