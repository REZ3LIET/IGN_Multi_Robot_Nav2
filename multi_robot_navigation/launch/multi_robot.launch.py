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


def generate_launch_description():
    ld = LaunchDescription()

    # Names and poses of the robots
    robots = [
        {'name': 'tb1', 'x_pose': '-1.5', 'y_pose': '-0.5', 'z_pose': 0.01},
        # {'name': 'tb2', 'x_pose': '-1.5', 'y_pose': '0.5', 'z_pose': 0.01},
        # {'name': 'tb3', 'x_pose': '1.5', 'y_pose': '-0.5', 'z_pose': 0.01},
        # {'name': 'tb4', 'x_pose': '1.5', 'y_pose': '0.5', 'z_pose': 0.01},
        # ...
        # ...
        ]

    TURTLEBOT3_MODEL = 'waffle'

    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',value=[
        os.path.join("/opt/ros/humble", "share"),
        ":" +
        os.path.join(get_package_share_directory('turtlebot3'), "models")])

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value=use_sim_time, description='Use simulator time'
    )

    turtlebot3_multi_robot = get_package_share_directory('multi_robot_navigation')
    nav_launch_dir = os.path.join(turtlebot3_multi_robot, 'launch', 'nav2_mod')

    sdf = os.path.join(
        get_package_share_directory('turtlebot3'),
        'models', 'turtlebot3', 'model.sdf')


    world_only = os.path.join(get_package_share_directory('turtlebot3'), "models", "worlds", "world_only.sdf")
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
                        get_package_share_directory('turtlebot3'),
                        "models", "worlds", "model.sdf"]),
                   '-allow_renaming', 'false'],
        )

    params_file = LaunchConfiguration('nav_params_file')
    declare_params_file_cmd = DeclareLaunchArgument(
        'nav_params_file',
        default_value=os.path.join(turtlebot3_multi_robot, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    
     
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_params_file_cmd)
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

    map_server=Node(package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': yaml_filename,
                     },],
        remappings=remappings)

    map_server_lifecyle=Node(package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server']}])


    ld.add_action(clock_bridge)
    # ld.add_action(map_server)
    # ld.add_action(map_server_lifecyle)

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
                   '-x', '-2.0',
                   '-y', '-0.5',
                   '-z', '0.01'],
        )

        bringup_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_launch_dir, 'bringup_launch.py')),
                    launch_arguments={  
                        'slam': 'False',
                        'namespace': namespace,
                        'use_namespace': 'True',
                        'map': yaml_filename,
                        'map_server': 'True',
                        'params_file': params_file,
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

        static_transform_publisher = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom",
            namespace=namespace,
            output="screen",
            arguments=['0', '0', '0', '0', '0', '0', "map", "odom"],
            remappings=remappings
        )

        if last_action is None:
            # Call add_action directly for the first robot to facilitate chain instantiation via RegisterEventHandler
            ld.add_action(turtlebot_state_publisher)
            ld.add_action(spawn_turtlebot3_burger)
            ld.add_action(bringup_cmd)
            ld.add_action(bridge)
            ld.add_action(static_transform_publisher)

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
                        bridge,
                        static_transform_publisher
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
        ld.add_action(declare_params_file_cmd)
    ######################

    return ld