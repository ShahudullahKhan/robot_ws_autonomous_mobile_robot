import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
# imports for delaying nodes
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    package_name='autonomous_mobile_robot' #<--- my package name

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','robot_state_pub.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )


    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    """ gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args':'--ros-args --params-file ' + gazebo_params_file}.items()
             ) """
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'use_sim_time': 'true',
            'world': os.path.join(get_package_share_directory(package_name), 'worlds', 'myrobot_world.world')
        }.items()
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'robot1'],
                        output='screen')

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )


    # Code for delaying node
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[diff_drive_spawner],
        )
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_broad_spawner],
        )
    )


    # RTAB-Map node
    rtabmap_params_file = os.path.join(get_package_share_directory(package_name),'config','rtabmap_params.yaml')

    # Create a namespace group for RTAB-Map related nodes
    rtabmap_namespace = 'rtabmap'

    rtabmap_group = GroupAction([
        PushRosNamespace(rtabmap_namespace),

        # IMU Filter node
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            output='screen',
            parameters=[{
                'use_mag': False,
                'world_frame': 'enu',
                'publish_tf': False,
                'use_sim_time': True
            }],
            remappings=[('/imu', '/imu/data')]
        ),

        # RTAB-Map Visual odometry node
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            parameters=[
                rtabmap_params_file,
                {'use_sim_time': True}
            ],
            remappings=[
                ('/imu', '/imu/data'),
                ('rgb/image', '/camera/color/image_raw'),
                ('depth/image', '/camera/depth/image_raw'),
                ('depth/camera_info', '/camera/depth/camera_info'),
                ('rgb/camera_info', '/camera/color/camera_info')
            ]
        ),

        # RTAB-Map SLAM node
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[
                rtabmap_params_file,
                {
                    'use_sim_time': True,
                    'frame_id': 'base_footprint',
                    'subscribe_depth': True,
                    'approx_sync': True
                }
            ],
            remappings=[
                ('rgb/image', '/camera/color/image_raw'),
                ('depth/image', '/camera/depth/image_raw'),
                ('depth/camera_info', '/camera/depth/camera_info'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('/imu', '/imu/data'),
                ('/rtabmap/mapData', 'mapData'),
                ('/rtabmap/cloud_map', 'cloud_map'),
            ],
            arguments=['-d']  # Delete database on start
        ),

        # RTAB-Map Visualization node
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[
                rtabmap_params_file,
                {'use_sim_time': True}
            ],
            remappings=[
                ('/imu', '/imu/data'),
                ('/rtabmap/mapData', 'mapData'),
                ('/rtabmap/cloud_map', 'cloud_map'),
                ('/rtabmap/mapGraph', 'mapGraph'),
                ('odom', 'odom')
            ]
        ),

         Node(
            package='rtabmap_util', executable='point_cloud_xyz', output='screen',
            parameters=[{'decimation': 2,
                         'max_depth': 3.0,
                         'voxel_size': 0.02}],
            remappings=[('depth/image', '/camera/depth/image_raw'),
                        ('depth/camera_info', '/camera/camera_info'),
                        ('cloud', '/camera/cloud')]),
        Node(
            package='rtabmap_util', executable='obstacles_detection', output='screen',
            parameters=[
                rtabmap_params_file,
                {'use_sim_time': True}
                ],
            remappings=[('cloud', '/camera/cloud'),
                        ('obstacles', '/camera/obstacles'),
                        ('ground', '/camera/ground')])
    ])
    
    
    # Delayed rtabmap_group execution
    delayed_rtabmap_group = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[rtabmap_group],
        )
    )  

    # Replace the diff_drive_spawner in the final return with delayed_diff_drive_spawner

    # Launch them all!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        rsp,
        gazebo,
        spawn_entity,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        delayed_rtabmap_group,
    ])
