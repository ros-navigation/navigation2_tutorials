## nav2_gps_waypoint_follower_demo

A tutorial package provided in `navigation2_tutorials` to showcase the usage of `FollowGPSWaypoints` action. 
`FollowGPSWaypoints` is an action interface in `nav2_waypoint_follower`, which directly accepts GPS(lat,long,alt) waypoints and navigates robot through them. You can see the Readme page of `nav2_waypoint_follower` package for more details of this action but 
in a glance, we can summarize this as: 
`robot_localization`'s `navsat_transform_node` provides a service named `fromLL`, which is used to convert pure GPS coordinates(longitude, latitude, alitude)
to cartesian coordinates in map frame(x,y), then the existent Follow Waypoints logic from `nav2_waypoint_follower` is used to get robot go through each converted waypoints. Finally this process is exposed as `FollowGPSWaypoins` action. 

This package includes a sample Gazebo world that was used to collect GPS waypoints. You can collect GPS waypoints given that your robot is attached with an GPS sensor that provides (lat,long,alt). Then you can subscribe to the GPS topic and save the waypoints to YAML file given in `params` folder of this package. 
e.g `ros2 topic echo /gps/fix`. 

The format of yaml file is expected to have; 

```yaml
waypoints: [wp0,wp1,wp2,wp3,wp4]
#lat, long, alt
wp0: [9.677703999088216e-07, -5.306676831178058e-05, 0.6442248001694679]
wp1: [9.677703999088216e-07, -5.306676831178058e-05, 0.6442248001694679]
wp2: [4.169383611283205e-05, -0.0006143364570898212, 0.6346865268424153]
wp3: [9.319715737387455e-05, -0.000620772355007051, 0.6348643703386188]
wp4: [8.37498018946476e-06, -2.402470336058297e-05, 0.6447164406999946]
```
where `waypoints` is basically a vector that includes parameters name for each waypoint. Note that correct format when entering a waypoints is:  `wpN:[lat, long, alt]`, all three variables should be `double` types. 

## Start the Gazebo world
Fist you need to make sure `GAZEBO_MODEL_PATH` IS set such that the specifc models this package uses are in on the path. Do this by; 

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/YOUR_USER_NAME/colcon_ws_rolling/src/navigation2_tutorials/nav2_gps_waypoint_follower_demo/models
```
```bash
ros2 launch nav2_gps_waypoint_follower_demo city_world.launch.py
```
The above command should start gazebo world, at this point you should figure out a way to spawn your robot into that world.

## Request waypoint following

The package has an executable node named `gps_waypoint_follower_demo` which reads the yaml file in `params` and creates a client to `FollowGPSWaypoins` action, then requests the waypoint following. You need to make sure, `waypoint_follower` lifecycle node is up and runnning as the  `FollowGPSWaypoins` action is exposed by that. 

You can launch this node by ;

```bash
ros2 launch nav2_gps_waypoint_follower_demo demo_gps_waypoint_follower.launch.py
```
