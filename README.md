# IGN_Multi_Robot_Nav2

**Multi-Robot Navigation using Ignition Gazebo and Turtlebot3**

This ROS 2 package integrates multi-robot navigation capabilities utilizing Ignition Gazebo and the Turtlebot3 platform. The package leverages **custom `nav2_bringup`** launch files to enable key functionalities such as navigation, localization, SLAM, and RViz visualization. It is compatible with **ROS 2 Humble**, **Gazebo Fortress**, and **Ubuntu 22.04**.

## Features

- Multi-robot navigation setup with Turtlebot3.
- Integration with Ignition Gazebo for simulated environments.
- Visualization in RViz for robot states and sensor data.

## Requirements

- **ROS 2 Humble**: Ensure that ROS 2 Humble is installed on your system.
- **Gazebo Fortress**: Use Gazebo Fortress for the simulation environment.
- **Ubuntu 22.04**: This package is tested on Ubuntu 22.04, ensuring full compatibility.

## Setup and Usage

### 1. Define Robots Configuration

You can configure and define the robots for the simulation in the [`multi_robot.launch.py`](./multi_robot_navigation/launch/multi_robot.launch.py) file. This file is located at:


Edit this file to specify the robot parameters and configuration.

### 2. Build and Launch the Simulation

To run the multi-robot simulation, follow these steps:

1. Navigate to your ROS workspace:

    ```bash
    cd ~/colcon_ws
    ```

2. Build the workspace:

    ```bash
    colcon build
    ```

3. Source the workspace setup:

    ```bash
    source install/setup.bash
    ```

4. Launch the simulation using the custom launch file:

    ```bash
    ros2 launch multi_robot_navigation multi_robot.launch.py
    ```

This will start the multi-robot simulation with the defined configurations.

## Acknowledgements

- **arshadlab/turtlebot3_multi_robot**: [GitHub Repository](https://github.com/arshadlab/turtlebot3_multi_robot)
- **Onicc/navigation2_ignition_gazebo_turtlebot3**: [GitHub Repository](https://github.com/Onicc/navigation2_ignition_gazebo_turtlebot3)

These projects were essential in the development and inspiration for the current implementation.
