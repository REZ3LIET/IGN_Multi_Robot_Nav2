#!/usr/bin/env python3
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Arshad Mehmood

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
import xacro
import yaml

def replace_namespace(d, old_str, new_str):
    if isinstance(d, dict):
        for key, value in d.items():
            d[key] = replace_namespace(value, old_str, new_str)
    elif isinstance(d, list):
        for i in range(len(d)):
            d[i] = replace_namespace(d[i], old_str, new_str)
    elif isinstance(d, str):
        return d.replace(old_str, new_str)
    return d

def replace_namespace_in_params(yaml_file_path, namespace):
    if not namespace.startswith('/'):
        namespace = '/' + namespace

    with open(yaml_file_path, 'r') as file:
        data = yaml.safe_load(file)
    
    # Replace namespace with 'namespace'
    data = replace_namespace(data, 'namespace', namespace)

    # Convert the modified data back to YAML format
    modified_yaml = yaml.dump(data, default_flow_style=False)

    # Write the modified YAML to the output file
    path = f"/tmp{namespace}.yaml"
    with open(path, 'w') as file:
        file.write(modified_yaml)

    print(f"Modified YAML has been saved to {path}")
    return path

def generate_launch_description():
    ld = LaunchDescription()

    # Names and poses of the robots
    robots = [
        {'name': 'tb1', 'x_pose': '-1.5', 'y_pose': '-0.5', 'z_pose': '0.01'},
        {'name': 'tb2', 'x_pose': '-1.5', 'y_pose': '0.5', 'z_pose': '0.01'},
        # {'name': 'tb3', 'x_pose': '1.5', 'y_pose': '-0.5', 'z_pose': '0.01'},
        # {'name': 'tb4', 'x_pose': '1.5', 'y_pose': '0.5', 'z_pose': '0.01'},
        # ...
        # ...
        ]

    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',value=[
        os.path.join("/opt/ros/humble", "share"),
        ":" +
        os.path.join(get_package_share_directory('multi_robot_navigation'), "models")])

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value=use_sim_time, description='Use simulator time'
    )

    turtlebot3_multi_robot = get_package_share_directory('multi_robot_navigation')
    nav_launch_dir = os.path.join(turtlebot3_multi_robot, 'launch', 'nav2_mod')

    sdf = os.path.join(
        get_package_share_directory('multi_robot_navigation'),
        'models', 'turtlebot3', 'model.sdf')


    world_only = os.path.join(get_package_share_directory('multi_robot_navigation'), "models", "worlds", "world_only.sdf")
    ign_gz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                              'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('ign_args', [' -r -v1 ' +
                              world_only
                             ])])

    ignition_spawn_world = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-file', PathJoinSubstitution([
                        get_package_share_directory('multi_robot_navigation'),
                        "models", "worlds", "model.sdf"]),
                   '-allow_renaming', 'false'],
        )

    params_file = os.path.join(turtlebot3_multi_robot, 'params', 'nav2_params.yaml')    
     
    ld.add_action(declare_use_sim_time)
    ld.add_action(ign_resource_path)
    ld.add_action(ign_gz)
    ld.add_action(ignition_spawn_world)
 
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    yaml_filename = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('nav2_bringup'),
            'maps',
            'turtlebot3_world.yaml'))
    
    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ]
    )

    ld.add_action(clock_bridge)

    ######################

    # Remapping is required for state publisher otherwise /tf and /tf_static 
    # will get be published on root '/' namespace
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    last_action = None
    # Spawn turtlebot3 instances in gazebo
    for robot in robots:

        namespace = '/' + robot['name']

        doc = xacro.parse(open(sdf))
        xacro.process_doc(doc, mappings={"namespace": namespace})
        robot_description = {"robot_description": doc.toxml()}

        # Create state publisher node for that instance
        turtlebot_state_publisher = Node(
            package='robot_state_publisher',
            name='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            remappings=remappings,
            output='both',
            parameters=[robot_description],
        )

        # Create spawn call
        spawn_turtlebot3_burger = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', f"{namespace}/robot_description",
                   '-name', robot['name'],
                   '-robot_namespace', namespace,
                   '-x', robot['x_pose'],
                   '-y', robot['y_pose'],
                   '-z', robot['z_pose']],
        )

        namespaced_param_file = replace_namespace_in_params(params_file, namespace)

        bringup_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_launch_dir, 'bringup_launch.py')),
                    launch_arguments={  
                        'slam': 'False',
                        'namespace': namespace,
                        'use_namespace': 'True',
                        'map': yaml_filename,
                        'map_server': 'True',
                        'params_file': namespaced_param_file,
                        'default_bt_xml_filename': os.path.join(
                            get_package_share_directory('nav2_bt_navigator'),
                            'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
                        'autostart': 'true',
                        'use_sim_time': use_sim_time,
                        'use_composition': 'False'
                    }.items()
                )

        bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            namespace=namespace,
            arguments=[
                f"{namespace}/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
                f"{namespace}/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
                f"{namespace}/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
                f"{namespace}/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
                f"{namespace}/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
                f"{namespace}/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model",
            ],
            output='screen'
        )

        if last_action is None:
            # Call add_action directly for the first robot to facilitate chain instantiation via RegisterEventHandler
            ld.add_action(turtlebot_state_publisher)
            ld.add_action(spawn_turtlebot3_burger)
            ld.add_action(bringup_cmd)
            ld.add_action(bridge)

        else:
            # Use RegisterEventHandler to ensure next robot creation happens only after the previous one is completed.
            # Simply calling ld.add_action for spawn_entity introduces issues due to parallel run.
            spawn_turtlebot3_event = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_action,
                    on_exit=[
                        spawn_turtlebot3_burger,
                        turtlebot_state_publisher,
                        bringup_cmd,
                        bridge
                    ],
                )
            )

            ld.add_action(spawn_turtlebot3_event)

        # Save last instance for next RegisterEventHandler
        last_action = spawn_turtlebot3_burger
    ######################

    ######################
    # Start rviz nodes and drive nodes after the last robot is spawned
    for robot in robots:

        namespace = [ '/' + robot['name'] ]

        # Create a initial pose topic publish call
        message = '{header: {frame_id: map}, pose: {pose: {position: {x: ' + \
            robot['x_pose'] + ', y: ' + robot['y_pose'] + \
            ', z: 0.1}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0000000}}, }}'

        initial_pose_cmd = ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '-t', '3', '--qos-reliability', 'reliable', namespace + ['/initialpose'],
                'geometry_msgs/PoseWithCovarianceStamped', message],
            output='screen'
        )

        rviz_config_file = os.path.join(get_package_share_directory("nav2_bringup"), 'rviz', 'nav2_default_view.rviz')
        rviz_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_launch_dir, 'rviz_launch.py')),
                launch_arguments={'use_sim_time': use_sim_time, 
                                  'namespace': namespace,
                                  'use_namespace': 'True',
                                  'rviz_config': rviz_config_file, 'log_level': 'warn'}.items()
                                    )

        # Use RegisterEventHandler to ensure next robot rviz launch happens 
        # only after all robots are spawned
        post_spawn_event = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=last_action,
                on_exit=[initial_pose_cmd, rviz_cmd],
            )
        )

        # Perform next rviz and other node instantiation after the previous intialpose request done
        last_action = initial_pose_cmd

        ld.add_action(post_spawn_event)
    ######################

    return ld