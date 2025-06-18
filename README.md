# Closed-Chain Affordance Planning for the BD Spot using ROS2
This repository contains the `cca_spot` package, which implements the closed-chain affordance planning framework on the Boston Dynamics Spot arm.

## Requirements

- `C++20`
- `ROS Humble`

## Dependencies

- **Robot Description** (for visualization)  
  Here, we use the `spot_description` package from the NRG `spot_ros` repository.

- **MoveIt Configuration** (for collision checking)  
  By default, we use the `spot_moveit_config` package from the `nrg_spot_manipulation` repository.

> **Tip:** To use your own robot description or MoveIt config, simply update the corresponding entries in the launch files under the `launch/` directory.


## Build and Install Instructions:
1. Install the closed-chain affordance planning libraries by following instructions from the following repository:
   [Link to instructions](https://github.com/UTNuclearRoboticsPublic/closed_chain_affordance_ros.git)

2. Clone this repository onto your machine ROS2 workspace `src` folder:
   ``` bash
   cd ~/<ros2_ws_name>/src
   ```
   ```bash
   git clone git@github.com:UTNuclearRoboticsPublic/closed_chain_affordance_spot.git
   ```

3. Build and source the `cca_spot` package:
   ```bash
   cd ~/<ros2_ws_name>
   ```
   ```bash
   colcon build --packages-select cca_spot
   ```
   ```bash
   source install/setup.bash
   ```

## Run Instructions:

### Using With a Physical Robot
To execute CCA-generated joint trajectories on the Spot robot:

1. Run the Spot driver:
   ```bash
   ros2 launch spot_bringup bringup.launch.py hostname:=192.168.50.3
   ```

2. Launch the CCA visualizer for Spot:

   ```bash
   ros2 launch cca_spot cca_spot_viz.launch.py
   ```

   This launches both the visualizer and an interactive RVIZ plugin for code-free planning and execution. To plan and execute trajectories using the RViz plugin, start the following action server:
   ```bash
   ros2 launch cca_spot cca_spot_action_server.launch.py
   ```

3. For programmatic task definition, launch the CCA planner which will plan for the tasks defined in `src/cca_spot_node.cpp`:
   ```bash
   ros2 launch cca_spot cca_spot.launch.py
   ```

### Using Without a Physical Robot
You can plan and visualize joint trajectories for the BD Spot using the CCA framework without needing a physical robot. The following demonstration showcases various CCA framework features on Spot.

1. Launch the offline state publisher for Spot for a desired named pose:
   ```bash
   ros2 launch spot_description offline_state_publisher.launch.py configuration:='unstowed'
   ```

1. Launch the CCA-visualizer for Spot:

   ```
   ros2 launch cca_spot cca_spot_viz.launch.py
   ```
   This launches both the visualizer and an interactive RVIZ plugin for code-free planning and execution. To plan and execute trajectories using the RViz plugin, start the following action server:
   ```bash
   ros2 launch cca_spot cca_spot_action_server.launch.py
   ```

2. Launch the CCA planner demo node:
   ```
   ros2 launch cca_spot cca_spot_demo.launch.py
   ```

You are encouraged to modify the tasks in the demo node to plan and visualize trajectories tailored to your specific applications. Task examples are also provided in the package README.md.

## Author
Janak Panthi aka Crasun Jans
