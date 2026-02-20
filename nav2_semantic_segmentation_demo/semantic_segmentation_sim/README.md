# Semantic Segmentation Simulation

Gazebo simulation environment for testing semantic segmentation with Nav2 integration.

## Overview

This package provides a complete simulation environment with:
- TurtleBot 4 robot model with RGBD camera
- Baylands world with sidewalk and grass terrain
- Nav2 integration with [semantic segmentation costmap layer](https://github.com/kiwicampus/semantic_segmentation_layer)
- RViz visualization configuration

The semantic segmentation inference is provided by the [`semantic_segmentation_node`](../semantic_segmentation_node/) package.

## Launch Files

### `nav2_segmentation_launch.py`
Main launch file that starts:
- Nav2 navigation stack with semantic segmentation costmap layer
- Semantic segmentation inference node (ONNX-based)
- RViz (optional)

```bash
ros2 launch semantic_segmentation_sim nav2_segmentation_launch.py rviz:=true
```

### `simulation_launch.py`
Starts the Gazebo simulation environment:
- Gazebo with Baylands world
- TurtleBot 4 robot spawn
- Static transforms

```bash
ros2 launch semantic_segmentation_sim simulation_launch.py headless:=false
```

### `segmentation_simulation_launch.py`
Complete launch file that starts both simulation and navigation:

```bash
ros2 launch semantic_segmentation_sim segmentation_simulation_launch.py
```

**Arguments:**
- `use_sim_gui` - Launch Gazebo with GUI (default: true)
- `use_rviz` - Launch RViz visualization (default: true)

**Example:**
```bash
# Launch without GUI and without RViz
ros2 launch semantic_segmentation_sim segmentation_simulation_launch.py use_sim_gui:=false use_rviz:=false
```

## Configuration

- `config/nav2_params.yaml` - Nav2 parameters with semantic segmentation layer config
- `config/segmentation_rviz_config.rviz` - RViz configuration for visualization
- `worlds/baylands.sdf` - Gazebo world file

## Semantic Segmentation Integration

The simulation uses a custom Nav2 costmap layer (`semantic_segmentation_layer`) that:
- Subscribes to segmentation masks, confidence, and point clouds
- Projects segmentation onto the costmap
- Assigns costs based on terrain class (sidewalk = traversable, grass = danger)

See `config/nav2_params.yaml` for detailed layer configuration.

