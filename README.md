# warmup_project
# Driving in a Square
## Description
In the first part of the project I wrote a script to drive the Turtlebot3 robot in a square. I did this by setting the forward velocity to a positive value for a set amount of time and then setting the angular velocity to a positive value for a set amount of time. The time the angular velocity was set to be positive was roughly as long as it took for the robot to turn 90 degrees. I did this process four times to drive the robot in a square shape.

## Code Explanation
I made an object `DriveSquare` that is a node that publishes ROS messages that drive the robot in a square shape. In the initialization, I initialize the ROS node, setup the publisher to publish to the cmd_vel ROS topic in the `twist_pub` attribute, and create a default twist message with all zeros in the `twist` attribute.

The `drive_straight()` method sets `linear.x` (forward velocity) to `0.2` and publishes it. Then `linear.x` is set to `0.0`, but `rospy.sleep(2)` causes 2 seconds to pass before the Twist message to stop moving is published.

The `turn_90_degrees()` method sets `angular.z` (angular velocity) to `0.6` and publishes it. Then `angular.z` is set to `0.0`, but `rospy.sleep(2.75)` causes 2.75 seconds to pass before the Twist message to stop turning is published.

The `run()` method calls `rospy.sleep(1)` to allow the publisher enough time to set up before publishing the first message. Then `drive_straight()` and `turn_90_degrees()` are called in a for-loop that repeats four times.

Lastly, in the main function, the ROS node is declared and ran using `node.run()`.
## gif
![drive_square.gif](https://github.com/KendrickXie/warmup_project/blob/main/gifs/drive_square.gif)

# Person Follower
## Description
For the second part of the project I wrote a script to make the Turtlebot3 robot follow a person. I did this by finding the angle and distance of the closest object. The robot then turns until the closest object is within the front 9 degrees of the robot and drives forward unless the closest distance is less than a certain distance.
## Code Explanation
I made an object `FollowPerson` that is a node that publishes ROS messages that makes the robot follow a person. In the initialization, I initialize the ROS node and declare the node as a subscriber to the scan ROS topic with `self.drive_to_closest` as the callback function. I then setup the publisher to publish to the cmd_vel ROS topic in the `twist_pub` attribute, and create a default twist message with all zeros in the `twist` attribute.

The `drive_to_closest(data)` method loops through the list of ranges from `data` to find the angle and distance of the object closest to the robot (`closest_angle` and `closest_angle_range`). If the closest object is from 5 degrees to 179 degrees of the robot (to the left), then angular velocity is set to turn left. If the closest object is from 180 degrees to 355 degrees of the robot (to the right), then angular velocity is set to turn right. If the angle is within the front 9 degrees of the robot, then angular velocity is set to stop turning. If `closest_angle_range` is greater `distance = 0.4`, then linear velocity is set to move foward. Otherwise it is set to stop moving. Then the Twist message is published.

The `run()` method calls `rospy.spin()` to keep the program alive.

Lastly, in the main function, the ROS node is declared and ran using `node.run()`.
## gif
![person_follower.gif](https://github.com/KendrickXie/warmup_project/blob/main/gifs/person_follower.gif)

# Wall Follower
## Description
For the third part of the project I wrote a script to make the Turtlebot3 robot follow a wall on its left side. I did this by driving forward until an object is detected. Then the robot drives towards the wall until it is within a maximum distance. If the robot gets within a minimum distance of the wall, the robot will drive away from the wall until its further than the minimum distance. When the robot is within a correct distance from the wall, it will drive straight while adjusting its angular velocity if the closest point is not with 87-93 degrees on the left side of the robot.
## Code Explanation
I made an object `FollowWall` that is a node that publishes ROS messages that makes the robot follow a wall on its left side. In the initialization, I initialize the ROS node and declare the node as a subscriber to the scan ROS topic with `self.follow_wall` as the callback function. I then setup the publisher to publish to the cmd_vel ROS topic in the `twist_pub` attribute, and create a default twist message with all zeros in the `twist` attribute.

The `follow_wall(data)` method sets linear velocity to move forward. It then loops through the list of ranges from `data` to find the angle and distance of the object closest to the robot (`closest_angle` and `closest_angle_range`). If no object is found, then angular velocity is set to go straight. Once an object is detected, to get to the right distance from the wall, if the closest point detected is above `max_distance`, the robot will stop moving forward, face about 45 degrees to the right of the point, and then drive forward. If the closest point detected is below `min_distance`, the robot will stop moving forward, face about 135 degrees to the right of the point, and then drive forward. To drive straight along a wall, if the closest point is within 87-93 degrees to the left of the robot, the robot will drive straight. Otherwise, if the closest point is in about the front 180 degrees of the robot, the robot will turn right. And if the closest point is in about the back 180 degrees of the robot, the robot will turn left. If none of the conditions are met, then the robot will go straight.

The `run()` method calls `rospy.spin()` to keep the program alive.

Lastly, in the main function, the ROS node is declared and ran using `node.run()`.
## gif
![wall_follower.gif](https://github.com/KendrickXie/warmup_project/blob/main/gifs/wall_follower.gif)

# Challenges
The most challenging part of driving the robot in a square was finding the right amount of time to turn in order to make a 90 degree turn. I was able to make the robot turn roughly 90 degrees by using a lower angular velocity and experimenting with different times. I did not run in to any major challenges when making the robot follow a person. When making the robot follow a wall, handling the edge cases of concave and convex corners was the most challenging part. To solve this problem I used if statements that keep the robot with the minimum and maximum distances from the wall.

# Future Work
For driving in a square, the timing to turn 90 degrees could be tuned to be more precise. For following a person, it would be interesting to make the robot go backwards if a person in front of it moves forward. For following a wall, if I had more time, I would make the robot follow the wall with less turning adjustments.

# Takeaways
• The robots do not always behave the same in simulation as they do in real life. For this reason it is important to come in person early enough to debug any unexpected issues. For example, the robot in simulation needed to turn for a different amount of time than the physical robot to turn about 90 degrees.  

• Although `rostopic ehco` is very useful, printing can still be very helpful when debugging. For example, when making the robot follow a wall, if the robot got stuck doing a certain behavior, it was helpful to print the behavior the robot was performing to find which behavior was the issue.
