#!/usr/bin/env python3

from turtle import distance
import rospy

# msg needed for /scan.
from sensor_msgs.msg import LaserScan

# msgs needed for /cmd_vel
from geometry_msgs.msg import Twist, Vector3

# How close we will get to a wall from the side.
side_distance = 2.0

# How close we will get to a wall from the front.
front_distance = 0.7

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

        # too_close = False
        # angle = 0

        # for angle_range in data.ranges:
        #     if angle_range < distance:
        #         too_close = True
        #         break
        #     angle = angle + 1
        #     if angle > 179:
        #         break
        
        # if too_close:
        #     # turn right
        #     self.twist.linear.x = -0.05
        #     self.twist.angular.z = 0.0
        # # if something is in front of the robot


        # elif data.ranges[45] > distance or data.ranges[135] > distance:
        #     #turn left
        #     print("left too far")
        #     self.twist.angular.z = 0.1
        
        # Loop through the list of ranges to find if there is any
        #   object detected within range
        any_object = False
        for angle_range in data.ranges:
            if angle_range != 0.0:
                any_object = True
                break

        
        # If there is no object, drive forward.
        if not any_object:
            self.twist.angular.z = 0.0
        # If too close to an object in front, stop moving foward
        #   and turn right.
        elif data.ranges[0] != 0.0 and data.ranges[0] < front_distance:
            print("right 1")
            #turn right
            self.twist.linear.x = 0.0
            self.twist.angular.z = -0.5
        # If the range at 45 degrees or 135 degrees is greater than
        #   side_distance, turn left.
        elif data.ranges[45] > side_distance and data.ranges[135] > side_distance and any_object: #TODO: could probably comment out any_object also try OR
            print("left 3")
            # turn left
            self.twist.angular.z = 0.2
        # If the ranges at 45 degrees and 135 degrees to the left of the robot
        #   are both less than side_distance, within 0.05 of each other,
        #   and are not 0, go straight.
        elif (#data.ranges[45] < distance and data.ranges[135] < distance
                data.ranges[45] - 0.05 < data.ranges[135]
                and data.ranges[45] + 0.05 > data.ranges[135]
                and data.ranges[45] != 0.0 and data.ranges[135] != 0.0):
            print("straight 1")
            # go straight
            self.twist.angular.z = 0.0
        # If nothing is detected at 45 degrees, but something is detected at
        #   135 degrees, turn left.
        elif data.ranges[45] == 0.0 and data.ranges[135] != 0.0:
            print("left 1")
            #turn left
            self.twist.angular.z = 0.2
        # If the range at 45 degrees is greater than at 135 degrees, turn left.
        elif (data.ranges[45] > data.ranges[135] and data.ranges[135] != 0.0): #or (data.ranges[45] == 0.0 and data.ranges[135] != 0.0):
            print("left 2")
            # turn left
            self.twist.angular.z = 0.2
        # If the range at 45 degrees is less than at 135 degrees, turn right.
        elif (data.ranges[45] < data.ranges[135] and data.ranges[45] != 0.0): #or (data.ranges[45] != 0.0 and data.ranges[135] == 0.0):
            print("right 2")
            # turn right
            self.twist.angular.z = -0.2
        # If nothing is detected at 90 degrees or 135 degrees, turn left.
        elif (data.ranges[60] > side_distance or data.ranges[60] == 0.0) and any_object:#(data.ranges[45] > side_distance or data.ranges[45] == 0.0):
            #turn left
            print("left 4")
            self.twist.angular.z = 0.5
        # Otherwise go straight.
        else:
            print("straight 2")
            # go straight
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
