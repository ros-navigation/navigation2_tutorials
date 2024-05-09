# nav2_pure_pursuit_controller

This repository contains the nav2_pure_pursuit_controller package, which is referenced in the [Nav2 documentation](https://docs.nav2.org/).

## Overview

The `nav2_pure_pursuit_controller` package is a modified version of the `navigation2_tutorials` Foxy release, adapted to the Humble release.

## Configuration

To configure the controller parameters, you can edit the `controller_server` parameters in the ROS parameter server:

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_pure_pursuit_controller::PurePursuitController"
      debug_trajectory_details: True
      desired_linear_vel: 0.2
      lookahead_dist: 0.4
      max_angular_vel: 1.0
      transform_tolerance: 1.0
