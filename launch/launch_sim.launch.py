import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetParameter
from launch.conditions import IfCondition, UnlessCondition
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
    #rtabmap_params_file = os.path.join(get_package_share_directory(package_name),'config','rtabmap_params.yaml')

    localization = LaunchConfiguration('localization')
    parameters={
          'frame_id':'base_footprint',
          'odom_frame_id':'odom',
          'odom_tf_linear_variance':0.001,
          'odom_tf_angular_variance':0.001,
          'subscribe_depth':True,
          'subscribe_rgbd':True,
          'subscribe_scan':False,
          'approx_sync':True,
          'sync_queue_size': 10,
          # RTAB-Map's internal parameters should be strings
          'RGBD/NeighborLinkRefining': 'true',    # Do odometry correction with consecutive laser scans
          'RGBD/ProximityBySpace':     'true',    # Local loop closure detection (using estimated position) with locations in WM
          'RGBD/ProximityByTime':      'false',   # Local loop closure detection with locations in STM
          'RGBD/ProximityPathMaxNeighbors': '10', # Do also proximity detection by space by merging close scans together.
          'Reg/Strategy':              '0',       # 0=Visual, 1=ICP, 2=Visual+ICP
          'Vis/MinInliers':            '12',      # 3D visual words minimum inliers to accept loop closure
          'RGBD/OptimizeFromGraphEnd': 'false',   # Optimize graph from initial node so /map -> /odom transform will be generated
          'RGBD/OptimizeMaxError':     '4',       # Reject any loop closure causing large errors (>3x link's covariance) in the map
          'Reg/Force3DoF':             'true',    # 2D SLAM
          'Grid/FromDepth':            'false',   # Create 2D occupancy grid from laser scan
          'Mem/STMSize':               '30',      # increased to 30 to avoid adding too many loop closures on just seen locations
          'RGBD/LocalRadius':          '5',       # limit length of proximity detections
          #'Icp/CorrespondenceRatio':   '0.2',     # minimum scan overlap to accept loop closure
          #'Icp/PM':                    'false',
          #'Icp/PointToPlane':          'false',
          #'Icp/MaxCorrespondenceDistance': '0.15',
          #'Icp/VoxelSize':             '0.05'
        }
        
    remappings=[
        ('rgb/image',       '/camera/image_raw'),
        ('depth/image',     '/camera/depth/image_raw'),
        ('rgb/camera_info', '/camera/camera_info'),
        ('rgbd_image',     '/rtabmap/rgbd_image'),
        ]
        
    config_rviz = os.path.join(
        get_package_share_directory(package_name), 'ros2_vizualizer', 'view_Robot_rtabmap_slam_model.rviz'
    )

    # Create a namespace group for RTAB-Map related nodes
    rtabmap_namespace = 'rtabmap'

    rtabmap_group = GroupAction([
        PushRosNamespace(rtabmap_namespace),
        
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[parameters,
              {'rgb_image_transport':'compressed',
               'depth_image_transport':'compressedDepth',
               'approx_sync_max_interval': 0.02}],
            remappings=remappings
            ),

        # RTAB-Map Visual odometry node
        # Node(
        #     package='rtabmap_odom',
        #     executable='rgbd_odometry',
        #     name='rgbd_odometry',
        #     output='screen',
        #     parameters=[
        #         parameters,
        #         {'use_sim_time': True}
        #     ],
        #     remappings=remappings
        # ),
        

        # RTAB-Map SLAM node
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=[
                parameters,
                {
                    'use_sim_time': True,
                    'frame_id': 'base_footprint',
                    'subscribe_rgbd':True,
                    'subscribe_depth': True,
                    'approx_sync': True
                }
            ],
            remappings=remappings,
            arguments=['-d']  # Delete database on start
            ),
        
        # Localization mode:
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}
                        ],
            remappings=remappings
            ),

        # RTAB-Map Visualization node
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            output='screen',
            parameters=[
                parameters,
                {'use_sim_time': True}
            ],
            remappings=remappings
            ),

        Node(
            package='rtabmap_util', executable='point_cloud_xyz', output='screen',
            parameters=[{'decimation': 2,
                         'max_depth': 3.0,
                         'voxel_size': 0.02}],
            remappings=[('depth/image', '/camera/depth/image_raw'),
                        ('depth/camera_info', '/camera/camera_info'),
                        ('cloud', '/camera/cloud')]
            ),
        
        Node(
            package='rtabmap_util', executable='obstacles_detection', output='screen',
            parameters=[
                parameters,
                {'use_sim_time': True}
                ],
            remappings=[
                ('cloud', '/camera/cloud'),
                ('obstacles', '/camera/obstacles'),
                ('ground', '/camera/ground')]
            )
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
        DeclareLaunchArgument('use_sim_time', default_value='true',description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('rviz',         default_value='false', description='Launch RVIZ (optional).'),
        DeclareLaunchArgument('localization', default_value='false', description='Launch in localization mode.'),
        DeclareLaunchArgument('rviz_cfg', default_value=config_rviz,  description='Configuration path of rviz2.'),
        
        SetParameter(name='use_sim_time', value=True),
        
        
        rsp,
        gazebo,
        spawn_entity,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        delayed_rtabmap_group,
    ])
