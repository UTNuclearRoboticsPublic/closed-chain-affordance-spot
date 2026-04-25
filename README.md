# Closed-Chain Affordance Planning for the BD Spot using ROS2
This repository contains the `cca_spot` package, which implements the closed-chain affordance planning framework on the Boston Dynamics Spot robot.  The demonstration video of the underlying paper showcasing simulation and real-world tasks is available [here](https://www.youtube.com/watch?v=Ukv93hbNrOM). An interactive RViz plugin is also available as one of many ways to interface with the planner. See the [plugin demo videos](https://github.com/UTNuclearRoboticsPublic/closed-chain-affordance-ros/blob/main/cca_ros_rviz_plugin/README.md) for a glance at the framework's capabilities.

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

> **Tip:** If you'd like to use your own robot description or MoveIt config for spot, simply update the relevant entries in the `cca_spot_settings.py` module in the `launch/` directory.


## Installation 
1. Navigate to your ROS2 workspace `src` folder and clone the repository:
   ``` bash
   cd ~/<ros2_ws_name>/src
   ```
   ```bash
   git clone git@github.com:UTNuclearRoboticsPublic/closed_chain_affordance_spot.git
   ```

3. Build and source the `cca_spot` package:
   ```bash
   cd ~/<ros2_ws_name>
   colcon build --packages-select cca_spot --cmake-args -DCMAKE_BUILD_TYPE=Release
   source install/setup.bash
   ```

## Usage

### With Physical Robot
To execute CCA-generated joint trajectories on the Spot robot:

1. Run the Spot driver:
   ```bash
   ros2 launch spot_bringup bringup.launch.py hostname:=192.168.50.3 has_arm:=True kinematic_model:="mobile_manipulation"
   ```

2. Launch the CCA visualizer for Spot:

   ```bash
   ros2 launch cca_spot cca_spot_val_and_viz.launch.py launch_rviz:=true
   ```

   This launches both the visualizer and an interactive RVIZ plugin for code-free planning and execution. To plan and execute trajectories using the RViz plugin, start the following action server:
   ```bash
   ros2 launch cca_spot cca_spot_action_server.launch.py
   ```
   At this point, you should be able to interactively plan and execute trajectories using the Rviz plugin. 

3. Alternatively, for programmatic task definition, launch the CCA planner which will plan for the tasks defined in `src/cca_spot_node.cpp`:
   ```bash
   ros2 launch cca_spot cca_spot.launch.py
   ```

### Without Physical Robot
You can plan and visualize joint trajectories for the BD Spot using the CCA framework without needing a physical robot. 

1. Launch the Moveit fake controller to simulate trajectory execution:
   ```bash
   ros2 launch spot_moveit_config fake_controller.launch.py kinematic_model:="mobile_manipulation"
   ```

   > Alternatively, you may launch the Spot offline state publisher to visualize the robot in Rviz at a named configuration and to use this state for planning:
   > ```bash
   > ros2 launch spot_description offline_state_publisher.launch.py has_arm:=True kinematic_model:="mobile_manipulation" configuration:="unstowed"
   > ```

2. Launch the CCA visualizer for Spot:

   ```bash
   ros2 launch cca_spot cca_spot_val_and_viz.launch.py launch_rviz:=true 
   ```
   This launches both the visualizer and an interactive RVIZ plugin for code-free planning and execution. To plan and execute trajectories using the RViz plugin, start the following action server:
   ```bash
   ros2 launch cca_spot cca_spot_action_server.launch.py
   ```
   At this point, you should be able to interactively plan trajectories using the Rviz plugin. 

3. Alternatively, for programmatic task definition, launch the CCA planner which will plan for the tasks defined in `src/cca_spot_node.cpp`:
   ```bash
   ros2 launch cca_spot cca_spot.launch.py
   ```

4. You may also run the demo by launching the CCA planner demo node:
   ```
   ros2 launch cca_spot cca_spot_demo.launch.py
   ```

## Task Examples

- Refer to the CCA-ROS `README.md` Code Tutorial section [here](https://github.com/UTNuclearRoboticsPublic/closed-chain-affordance-ros/blob/main/README.md) for various planning examples.

## Other Recommendations

- Explore the interactive RVIZ plugin for code-free planning and execution
- Modify demo node tasks to create custom trajectories

## Author

**Janak Panthi** (aka Crasun Jans)

## Support

For issues, feature requests, or contributions, please open an issue in the GitHub repository.
