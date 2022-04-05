# warmup_project
# Driving in a Square
## Description
In the first part of the project I wrote a script to drive the Turtlebot3 robot in a square. I did this by setting the forward velocity to a positive value for a set amount of time and then setting the angular velocity to a positive value for a set amount of time. The time the angular velocity was set to be positive was roughly as long as it took for the robot to turn 90 degrees. I did this process four times to drive the robot in a square shape.

## Code Explanation
I made an object `DriveSquare` that is a node that publishes ROS messages that drive the robot in a square shape. In the initialization, I initialize the ROS node, setup the publisher to publish to the cmd_vel ROS topic in the `twist_pub` attribute , and create a default twist message with all zeros in the `twist` attribute.

The `drive_straight()` method sets `linear.x` (forward velocity) to `0.2` and publishes it. Then `linear.x` is set to `0.0`, but `rospy.sleep(2)` causes 2 seconds to pass before the Twist message to stop moving is published.

The `turn_90_degrees()` method sets `angular.z` (angular velocity) to `0.6` and publishes it. Then `angular.z` is set to `0.0`, but `rospy.sleep(2.75)` causes 2.75 seconds to pass before the Twist message to stop turning is published.

The `run()` method calls `rospy.sleep(1)` to allow the publisher enough time to set up before publishing the first message. Then `drive_straight()` and `turn_90_degrees()` are called in a for-loop that repeats four times.

Lastly, in the main function, the ROS node is declared and ran using `node.run()`.
## gif
![drive_square.gif](https://github.com/KendrickXie/warmup_project/blob/main/gifs/drive_square.gif)
