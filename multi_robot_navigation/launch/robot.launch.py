import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']   # waffle

def generate_launch_description():
    namespace = "/tb1"
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world_name', default='turtlebot3_world')

    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',value=[
        os.path.join("/opt/ros/humble", "share"),
        ":" +
        os.path.join(get_package_share_directory('turtlebot3'), "models")])

    # Robot state publisher
    sdf = os.path.join(
        get_package_share_directory('turtlebot3'),
        'models', 'turtlebot3', 'model.sdf')

    doc = xacro.parse(open(sdf))
    xacro.process_doc(doc)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'robot_description': doc.toxml()
            }
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # Spawn robot
    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-entity', TURTLEBOT3_MODEL,
                   '-name', TURTLEBOT3_MODEL,
                   '-file', PathJoinSubstitution([
                        get_package_share_directory('turtlebot3'),
                        "models", "turtlebot3", "model.sdf"]),
                   '-allow_renaming', 'true',
                   '-x', '-2.0',
                   '-y', '-0.5',
                   '-z', '0.01'],
        )
    
    # Spawn world
    ignition_spawn_world = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-file', PathJoinSubstitution([
                        get_package_share_directory('turtlebot3'),
                        "models", "worlds", "model.sdf"]),
                   '-allow_renaming', 'false'],
        )


    world_only = os.path.join(get_package_share_directory('turtlebot3'), "models", "worlds", "world_only.sdf")

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'expand_gz_topic_names': True
            }
        ],
        arguments=[
                # Velocity command (ROS2 -> IGN)
                'cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                # Odometry (IGN -> ROS2)
                'odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                # TF (IGN -> ROS2)
                'odom/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                # Clock (IGN -> ROS2)
                'clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                # Joint states (IGN -> ROS2)
                'joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
                # Lidar (IGN -> ROS2)
                'scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                'scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
                # IMU (IGN -> ROS2)
                'imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                # Camera (IGN -> ROS2)
                'camera/rgb/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
                'camera/rgb/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
                ],
        remappings=[
            ("odom/tf", "tf")
        ],
        output='screen'
    )

    # Nav2 Bringup
    yaml_filename = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('nav2_bringup'),
            'maps',
            'turtlebot3_world.yaml'))

    map_server = Node(package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {
                'yaml_filename': yaml_filename,
            }
        ]
    )

    map_server_lifecyle = Node(package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server']}])

    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('multi_robot_navigation'),
            'params',
            'nav2_params.yaml'))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('multi_robot_navigation'), 'launch')
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_file_dir, 'nav2_mod/bringup_launch.py')),
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
            # 'log_level': 'warn'
        }.items(),
    )

    static_transform_publisher = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen" ,
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]
        )

    rviz_config_file = os.path.join(get_package_share_directory("nav2_bringup"), 'rviz', 'nav2_default_view.rviz')
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_file_dir, 'nav2_mod/rviz_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time, 
                'namespace': namespace,
                'use_namespace': 'True',
                'rviz_config': rviz_config_file,
                'log_level': 'warn'
            }.items()
        )

    return LaunchDescription([
        ign_resource_path,
        ignition_spawn_entity,
        ignition_spawn_world,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                              'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('ign_args', [' -r -v1 ' +
                              world_only
                             ])]),
                             
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),

        DeclareLaunchArgument(
            'world_name',
            default_value=world_name,
            description='World name'),

        robot_state_publisher,
        # PushRosNamespace(namespace),
        # map_server,
        # map_server_lifecyle,
        nav2_bringup,
        bridge,
        static_transform_publisher,
        rviz_cmd
    ])
