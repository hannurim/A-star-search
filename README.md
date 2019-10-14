# A star search
Operates turtlebot3 in ROS

It creates the path of the robot on the map using A* search algorithm.

You have to roslaunch turtlebot3_navigation, rosrun goal_publisher and rosrun astar.
Set the initial pose on the rviz.
Publish final goal at the goal_publisher node to give commands in a different way than turtlebot3_navigation package.
Please make sure to subscribe goal after subscribing map and initial pose.

If astar node completes the subscribe map, initial pose and final goal,
It will calculates the cost as much as the value you set.

    COST = EUCLIDEAN DISTANCE + INTRINSIC COST + PREVIOUS DIRECTION

Choosing the minimum cost from the four directions(up, down, left, right).
Saving the intermediate goals from initial pose to final goal step by step.

I can't handle the cmd_vel command very well. Also I can't understand the quaternion yet.
So, Just publish the intermediate destinations using action client library.
Orientation always same as the beginning.
Finally, The robot will arrive to the final goal.

I failed to implement a countermeasure that destinations overlap with obstacles.
