# Closed-Chain Affordance Planning for the BD Spot using ROS2
This repository contains the `cca_spot_arm` package, which implements the closed-chain affordance planning framework on the Boston Dynamics Spot arm.

## Requirements

- `C++20`
- `ROS Humble`

## Dependencies
- [CCA Libraries](https://github.com/UTNuclearRoboticsPublic/closed_chain_affordance.git)
- [CCA ROS Interface](https://github.com/UTNuclearRoboticsPublic/closed_chain_affordance_ros.git)
- [`spot_ros`](https://github.com/UTNuclearRoboticsPublic/spot_ros.git)
- [`spot_manipulation`](https://github.com/UTNuclearRoboticsPublic/spot_manipulation.git)

If you're not using a real robot, from `spot_ros` and `spot_manipulation`, it's sufficient to build only the `spot_description` and `spot_moveit_config` packages, which are used for visualization and collision checking:
```bash
colcon build --packages-select spot_description spot_moveit_config
```
Otherwise, build all packages with:
```bash
colcon build
```

> **Tip:** If you'd like to use your own robot description or MoveIt config for the spot arm, simply update the corresponding entries in the launch files under the `launch/` directory.


## Installation 
1. Navigate to your ROS2 workspace `src` folder and clone the repository:
   ``` bash
   cd ~/<ros2_ws_name>/src
   ```
   ```bash
   git clone git@github.com:UTNuclearRoboticsPublic/closed_chain_affordance_spot.git
   ```

3. Build and source the `cca_spot_arm` package:
   ```bash
   cd ~/<ros2_ws_name>
   colcon build --packages-select cca_spot_arm --cmake-args -DCMAKE_BUILD_TYPE=Release
   source install/setup.bash
   ```

## Usage

### With Physical Robot
To execute CCA-generated joint trajectories on the Spot robot:

1. Run the Spot driver:
   ```bash
   ros2 launch spot_bringup bringup.launch.py hostname:=192.168.50.3
   ```

2. Launch the CCA visualizer for Spot:

   ```bash
   ros2 launch cca_spot_arm cca_spot_arm_viz.launch.py
   ```

   This launches both the visualizer and an interactive RVIZ plugin for code-free planning and execution. To plan and execute trajectories using the RViz plugin, start the following action server:
   ```bash
   ros2 launch cca_spot_arm cca_spot_arm_action_server.launch.py
   ```
   At this point, you should be able to interactively plan and execute trajectories using the Rviz plugin. 

3. Alternatively, for programmatic task definition, launch the CCA planner which will plan for the tasks defined in `src/cca_spot_arm_node.cpp`:
   ```bash
   ros2 launch cca_spot_arm cca_spot_arm.launch.py
   ```

### Without Physical Robot
You can plan and visualize joint trajectories for the BD Spot using the CCA framework without needing a physical robot. The following demonstration showcases various CCA framework features on Spot.

1. Launch the offline state publisher for Spot for a desired named pose:
   ```bash
   ros2 launch spot_description offline_state_publisher.launch.py configuration:='unstowed'
   ```

2. Launch the CCA-visualizer for Spot:

   ```
   ros2 launch cca_spot_arm cca_spot_arm_viz.launch.py
   ```
   This launches both the visualizer and an interactive RVIZ plugin for code-free planning and execution. To plan and execute trajectories using the RViz plugin, start the following action server:
   ```bash
   ros2 launch cca_spot_arm cca_spot_arm_action_server.launch.py
   ```
   At this point, you should be able to interactively plan trajectories using the Rviz plugin. 

3. Alternatively, launch the CCA planner demo node:
   ```
   ros2 launch cca_spot_arm cca_spot_arm_demo.launch.py
   ```

## Task Examples

- Refer to the package's `README.md` [here](./cca_spot_arm/README.md) for various planning examples.

## Other Recommendations

- Explore the interactive RVIZ plugin for code-free planning and execution
- Modify demo node tasks to create custom trajectories

## Author

**Janak Panthi** (aka Crasun Jans)

## Support

For issues, feature requests, or contributions, please open an issue in the GitHub repository.
