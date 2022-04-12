#!/usr/bin/env python3

from turtle import distance
import rospy

# msg needed for /scan.
from sensor_msgs.msg import LaserScan

# msgs needed for /cmd_vel
from geometry_msgs.msg import Twist, Vector3

# How close we will get to a wall from the side.
max_distance = 0.5

min_distance = 0.3

# How close we will get to a wall from the front.
#front_distance = 0.7

class FollowWall(object):
    """ This node publishes ROS messages making the robot follow a wall on its left side """

    def __init__(self):
        # initialize the ROS node
        rospy.init_node('drive_square')

        # Declare our node as a subscriber to the scan topic and
        #   set self.follow_wall as the function to be used for callback.
        rospy.Subscriber("/scan", LaserScan, self.follow_wall)

        # Setup publisher to the cmd_vel ROS topic.
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Create a default Twist msg with values of all zeros.
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin,angular=ang)

    def follow_wall(self, data):
        # Set Twist message to move forward
        self.twist.linear.x = 0.05
        
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

       
        # If there is no object, drive forward.
        if closest_angle_range == 100.0:
            print("no object: straight")
            self.twist.angular.z = 0.0
        # If the closest object is outside of max_distance,
        #   turn about 45 degrees to the right of it,
        #   and if already in the right direction, go forward.
        elif closest_angle_range > max_distance:
            print("object too far: drive to object")
            # If the closest angle is on the left or back side,
            #   stop moving forward and turn left.
            if closest_angle >= 50 and closest_angle <= 225:
                self.twist.angular.z = 0.6
                self.twist.linear.x = 0.00
            # If the closest angle is on the right or front side,
            #   stop moving forward and turn right.
            elif closest_angle < 40 or closest_angle > 225:
                self.twist.angular.z = -0.6
                self.twist.linear.x = 0.00
            # If the closest angle is about 45 degrees to the left
            #   of the robot, stop turning and go forward.
            else:
                self.twist.angular.z = 0.0
        # If the closest object is under min_distance,
        #   turn about 135 degrees to the right of it,
        #   and if already in the right direction, go forward.
        elif closest_angle_range < min_distance:
            print("object too close: drive away")
            # If the closest angle is on the left or front side,
            #   stop moving forward and turn right.
            if closest_angle < 130 or closest_angle > 315:
                self.twist.angular.z = -0.6
                self.twist.linear.x = 0.00
            # If the closest angle is on the right or back side,
            #   stop moving forward and turn left.
            elif closest_angle > 140 and closest_angle <= 315:
                self.twist.angular.z = 0.6
                self.twist.linear.x = 0.00
            # If the closest angle is about 135 degrees to the left
            #   of the robot, stop turning and go forward.
            else:
                self.twist.angular.z = 0.0
        # If the closest angle is about 90 degrees to the left
        #   of the robot, go straight.
        elif closest_angle >= 87 and closest_angle <= 93:
            self.twist.angular.z = 0.0
        # If the closest angle is within about the front 180 degrees
        #   of the robot, turn right.
        elif closest_angle < 87 or closest_angle >= 270:
            self.twist.angular.z = -0.2
        # If the closest angle is within about the back 180 degrees
        #   of the robot, turn left.
        elif closest_angle > 93 and closest_angle < 270:
            self.twist.angular.z = 0.2
        # If none of the conditions are met, go straight.
        else:
            self.twist.angular.z = 0.0
        
        
        # Publish the Twist message
        self.twist_pub.publish(self.twist)

    def run(self):
        # Keep the program alive.
        rospy.spin()
            
if __name__ == '__main__':
    # declare the ROS node and run it
    node = FollowWall()
    node.run()
