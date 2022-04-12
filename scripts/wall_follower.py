#!/usr/bin/env python3

from dis import dis
import rospy

# msg needed for /scan.
from sensor_msgs.msg import LaserScan

# msgs needed for /cmd_vel
from geometry_msgs.msg import Twist, Vector3

# How close we will get to a person.
distance = 0.1
front_distance = 0.7

class FollowWall(object):
    """ This node publishes ROS messages driving the robot in a square shape """

    def __init__(self):
        # initialize the ROS node
        rospy.init_node('drive_square')

        # Declare our node as a subscriber to the scan topic and
        #   set self.process_scan as the function to be used for callback.
        rospy.Subscriber("/scan", LaserScan, self.follow_wall)

        # setup publisher to the cmd_vel ROS topic
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # create a default twist msg with values of all zeros
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin,angular=ang)

    def follow_wall(self, data):
        # go forward
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

        any_object = False
        for angle_range in data.ranges:
            if angle_range != 0.0:
                any_object = True
                break

        
        if not any_object:
            self.twist.angular.z = 0.0
        elif data.ranges[0] != 0.0 and data.ranges[0] < front_distance:
            print("right 1")
            #turn right
            self.twist.linear.x = 0.0
            self.twist.angular.z = -0.5
        elif (#data.ranges[45] < distance and data.ranges[135] < distance
                data.ranges[45] - 0.05 < data.ranges[135]
                and data.ranges[45] + 0.05 > data.ranges[135]
                and data.ranges[45] != 0.0 and data.ranges[135] != 0.0):
            print("straight 1")
            # go straight
            self.twist.angular.z = 0.0
        elif data.ranges[45] == 0.0 and data.ranges[135] != 0.0:
            print("left 1")
            #turn left
            self.twist.angular.z = 0.2
        elif (data.ranges[45] > data.ranges[135] and data.ranges[135] != 0.0): #or (data.ranges[45] == 0.0 and data.ranges[135] != 0.0):
            print("left 2")
            # turn left
            self.twist.angular.z = 0.2
        elif (data.ranges[45] < data.ranges[135] and data.ranges[45] != 0.0): #or (data.ranges[45] != 0.0 and data.ranges[135] == 0.0):
            print("right 2")
            # turn right
            self.twist.angular.z = -0.2
        elif data.ranges[45] > distance and data.ranges[135] > distance and any_object:
            print("left 3")
            # turn left
            self.twist.angular.z = 0.2
        elif data.ranges[45] == 0.0 or data.ranges[90] == 0.0:
            #turn left
            print("left 4")
            self.twist.angular.z = 0.2
        else:
            print("straight 2")
            # go straight
            self.twist.angular.z = 0.0
        
        #publish the Twist message
        self.twist_pub.publish(self.twist)

    def run(self):
        # Keep the program alive.
        rospy.spin()
            
if __name__ == '__main__':
    # declare the ROS node and run it
    node = FollowWall()
    node.run()