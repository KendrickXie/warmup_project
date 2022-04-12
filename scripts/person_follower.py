#!/usr/bin/env python3

import rospy

# msg needed for /scan.
from sensor_msgs.msg import LaserScan

# msgs needed for /cmd_vel
from geometry_msgs.msg import Twist, Vector3

# How close we will get to a person.
distance = 0.4

class FollowPerson(object):
    """ This node publishes ROS messages making the robot follow a person """

    def __init__(self):
        # initialize the ROS node
        rospy.init_node('follow_person')

        # Declare our node as a subscriber to the scan topic and
        #   set self.drive_to_closest as the function to be used for callback.
        rospy.Subscriber("/scan", LaserScan, self.drive_to_closest)

        # Setup publisher to the cmd_vel ROS topic.
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Create a default Twist msg with values of all zeros.
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin,angular=ang)

    def drive_to_closest(self, data):
        # Loop through the list of ranges to find the closest angle
        #   and the closest angle's range.
        closest_angle = 0
        closest_angle_range = 100.0
        angle = 0
        for angle_range in data.ranges:
            if angle_range < closest_angle_range and angle_range != 0:
                closest_angle_range = angle_range
                closest_angle = angle
            angle = angle + 1
        

        # If the closest angle is on the left, turn left.
        if closest_angle >= 5 and closest_angle <= 179:
            self.twist.angular.z = 0.6
        # If the closest angle is on the right, turn right.
        elif closest_angle >= 180 and closest_angle <= 355:
            self.twist.angular.z = -0.6
        # If the angle is within the 4 degrees on either
        #   side of the front of the robot, stop turning.
        else:
            self.twist.angular.z = 0.0

        
        # Go forward if not close enough to person.
        if closest_angle_range >= distance:   
            self.twist.linear.x = 0.1
        # Close enough to person, stop.
        else:
            self.twist.linear.x = 0
        

        # Publish the Twist message
        self.twist_pub.publish(self.twist)

    def run(self):
        # Keep the program alive.
        rospy.spin()
            
if __name__ == '__main__':
    # declare the ROS node and run it
    node = FollowPerson()
    node.run()
