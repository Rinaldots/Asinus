# Asinus Navigation Package

This package provides navigation capabilities for the Asinus robot using the ROS 2 Navigation2 (Nav2) stack.

## Overview

The `asinus_nav` package contains launch files, configuration files, and RViz setups for autonomous navigation of the Asinus robot. It supports both SLAM (Simultaneous Localization and Mapping) and navigation with pre-built maps.

## Package Structure

```
asinus_nav/
├── launch/                 # Launch files
│   ├── asinus_nav.launch.py    # Main navigation launch file
│   └── asinus_slam.launch.py   # SLAM launch file
├── config/                 # Configuration files
│   ├── nav2_params.yaml        # Nav2 stack parameters
│   └── slam_params.yaml        # SLAM Toolbox parameters
├── rviz/                   # RViz configurations
│   ├── asinus_nav.rviz         # Navigation visualization
│   └── asinus_slam.rviz        # SLAM visualization
├── maps/                   # Map files directory
└── README.md
```

## Dependencies

This package depends on:

- `asinus_description`: Robot description
- `asinus_gz`: Gazebo simulation
- `ros2_control_asinus`: Robot control
- `navigation2`: Nav2 stack
- `slam_toolbox`: SLAM capabilities
- `robot_localization`: Extended Kalman Filter localization

## Usage

### SLAM (Mapping)

To create a map of the environment:

```bash
ros2 launch asinus_nav asinus_slam.launch.py use_sim:=true
```

This will:
- Launch the robot simulation (if `use_sim:=true`)
- Start SLAM Toolbox for mapping
- Open RViz with SLAM visualization

Drive the robot around using teleop or navigation commands to build the map. Save the map when satisfied:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

### Navigation

To navigate with a pre-built map:

```bash
ros2 launch asinus_nav asinus_nav.launch.py use_sim:=true map_file:=/path/to/your/map.yaml
```

This will:
- Launch the robot simulation (if `use_sim:=true`)
- Load the specified map
- Start the Nav2 stack
- Open RViz with navigation visualization

### Parameters

#### asinus_nav.launch.py

- `use_sim` (default: `true`): Whether to use simulation or real robot
- `map_file` (default: `""`): Path to the map file for navigation
- `use_slam` (default: `false`): Whether to use SLAM instead of localization
- `use_rviz` (default: `true`): Whether to start RViz

#### asinus_slam.launch.py

- `use_sim` (default: `true`): Whether to use simulation or real robot
- `use_rviz` (default: `true`): Whether to start RViz

## Configuration

### Navigation Parameters

The main navigation parameters are defined in `config/nav2_params.yaml`. Key parameters include:

- **Robot dimensions**: Configure `robot_radius` in costmap settings
- **Velocity limits**: Set in `controller_server` parameters
- **Planning**: Configure planners in `planner_server`
- **Costmaps**: Local and global costmap settings

### SLAM Parameters

SLAM parameters are in `config/slam_params.yaml`. Key settings:

- **Scan topic**: Default is `/scan`
- **Frame IDs**: `odom_frame`, `map_frame`, `base_frame`
- **Resolution**: Map resolution (default: 0.05m)
- **Loop closure**: Enabled by default

## Customization

### Adding New Planners

To add custom planners, modify the `planner_server` section in `nav2_params.yaml`:

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased", "MyCustomPlanner"]
    MyCustomPlanner:
      plugin: "my_package::MyCustomPlanner"
      # Custom parameters here
```

### Modifying Robot Footprint

Update the robot dimensions in both local and global costmap configurations:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      robot_radius: 0.22  # Adjust to your robot's size
```

## Troubleshooting

### Common Issues

1. **Robot not moving**: Check velocity limits and safety parameters
2. **Poor localization**: Adjust AMCL parameters or add more sensor data
3. **Planning failures**: Verify costmap configurations and obstacle detection

### Debug Topics

Monitor these topics for debugging:

- `/diagnostics`: System diagnostics
- `/local_costmap/costmap`: Local costmap data
- `/global_costmap/costmap`: Global costmap data
- `/plan`: Planned path
- `/cmd_vel`: Velocity commands

## License

This package is licensed under the BSD 3-Clause License. See the LICENSE file for details.
