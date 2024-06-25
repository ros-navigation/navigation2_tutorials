# Nav2 Pure pursuit controller
Tutorial code referenced in https://docs.nav2.org/plugin_tutorials/docs/writing_new_nav2controller_plugin.html

This controller implements a the pure pursuit algorithm to track a path.

## How the algorithm works
The global path is continuously pruned to the closest point to the robot (see the figure below).
Then the path is transformed to the robot frame and a lookahead point is determined.
This lookahead point will be given to the pure pursuite algorithm to calculate a command velocity.

![Lookahead algorithm](./doc/lookahead_algorithm.png)
