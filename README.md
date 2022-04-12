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
For the third part of the project I wrote a script to make the Turtlebot3 robot follow a wall on its left side. I did this by driving forward until an object is detected. The robot then drives forward while turning to follow a wall, unless an object is detected that is too close to the front. I compared the ranges detected at 45 degrees and 135 degrees to the left of the robot to adjust the robot's direction to have about equal distance from at wall at both angles. If there is something detected within a certain range in front of the robot it will turn right (for the case of a concave corner). If there is nothing detected at 90 degrees or 135 degrees to the left of the robot, it will turn left (for the case of a convex corner).
## Code Explanation
I made an object `FollowWall` that is a node that publishes ROS messages that makes the robot follow a wall on its left side. In the initialization, I initialize the ROS node and declare the node as a subscriber to the scan ROS topic with `self.follow_wall` as the callback function. I then setup the publisher to publish to the cmd_vel ROS topic in the `twist_pub` attribute, and create a default twist message with all zeros in the `twist` attribute.

The `follow_wall(data)` method sets linear velocity to move forward. It then loops through the list of ranges from `data` to find whether any object is detected (a boolean stored in `any_object`). If no object is detected, angular velocity is set to move straight. Otherwise, if an object is within distance `front_distance`, the linear velocity is set to stop moving and the angular velocity is set to turn right. Otherwise, the ranges values at `data.ranges[45]` and `data.ranges[135]` are checked and the angular velocity is adjusted accordingly to turn the robot either left or right. If none of the previous turn adjustment cases are met and nothing is detected at `data.ranges[90]` or `data.ranges[135]`, the angular velocity is set to turn. Otherwise, the angular velocity will be set to go straight. Then the Twist message is published.

The `run()` method calls `rospy.spin()` to keep the program alive.

Lastly, in the main function, the ROS node is declared and ran using `node.run()`.
## gif
![wall_follower.gif](https://github.com/KendrickXie/warmup_project/blob/main/gifs/wall_follower.gif)

# Challenges
The most challenging part of driving the robot in a square was finding the right amount of time to turn in order to make a 90 degree turn. I was able to make the robot turn roughly 90 degrees by using a lower angular velocity and experimenting with different times. I did not run in to any major challenges when making the robot follow a person. When making the robot follow a wall, handling the edge cases of concave and convex corners was the most challenging part. I did this by introducing if statements for handling those cases.

# Future Work
For driving in a square, the timing to turn 90 degrees could be tuned to be more precise. For following a person, it would be interesting to make the robot go backwards if a person in front of it moves forward. For following a wall, the distance the robot is from the wall when turning corners could be improved to be more constant.

# Takeaways
• The robots do not always behave the same in simulation as they do in real life. For this reason it is important to come in person early enough to debug any unexpected issues. For example, the robot in simulation needed to turn for a different amount of time than the physical robot to turn about 90 degrees.
• Although `rostopic ehco` is very useful, printing can still be very helpful when debugging. For example, when making the robot follow a wall, if the robot got stuck doing a certain behavior, it was helpful to print the behavior the robot was performing to find which behavior is the issue.
