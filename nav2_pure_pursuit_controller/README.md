# Nav2 Pure pursuit controller

Tutorial code example is referenced in https://navigation.ros.org/plugin_tutorials/docs/writing_new_nav2controller_plugin.html

This controller implements a the pure pursuit algorithm to track a path.

## How the algorithm works
The global path is continuously pruned to the closest point to the robot (see the figure below).
Then the path is transformed to the robot frame and a lookahead point is determined.
This lookahead point will be given to the pure pursuite algorithm to calculate a command velocity.

![Lookahead algorithm](./doc/lookahead_algorithm.png)

## Features

This implementation has a number of non-standard features.
- Collision avoidance in the computed velocity direction between the robot in the look ahead point to ensure safe path following. Uses a maximum collision time parameter to inform the amount of time to forward simulate for collisions. Set to a very large number to always forward simulate to the carrot location
- Optional dynamic scaling of the look ahead point distance proportional to velocity. This helps you have a more stable robot path tracking over a broader range of velocity inputs if your robot has a large band of operating velocities. There are parameters for the minimum and maximum distances as well.
- Optional slow on approach to the goal. The default algorithm tracks a path at a given linear velocity. This feature allows you to slow the robot on its approach to a goal and also set the minimum percentage (from 0.0-1.0) on approach so it doesn't approach 0% before attaining the goal.
- Kinematic speed limiting on linear velocities to make sure the resulting trajectories are kinematically feasible on getting up to speed and slowing to a stop.

This package will also publish a few useful topics:
- lookahead point: the position at which the lookahead point is located the robot is attempting to follow
- lookahead arc: the arc produced by the pure pursuit algorithm for debug visualization while tuning. It is also the points on the path that the collision detector is running. In a collision state, the last published arc will be the points leading up to, and including, the first point in collision.
