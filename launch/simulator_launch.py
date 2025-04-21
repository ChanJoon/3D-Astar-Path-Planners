from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('init_x', default_value='-0.0'),
        DeclareLaunchArgument('init_y', default_value='-0.0'),
        DeclareLaunchArgument('init_z', default_value='0.0'),
        DeclareLaunchArgument('obj_num', default_value='1'),
        DeclareLaunchArgument('map_size_x_', default_value='40'),
        DeclareLaunchArgument('map_size_y_', default_value='40'),
        DeclareLaunchArgument('map_size_z_', default_value='10'),
        DeclareLaunchArgument('c_num', default_value='0'),
        DeclareLaunchArgument('p_num', default_value='80'),
        DeclareLaunchArgument('odom_topic', default_value='/visual_slam/odom'),
        DeclareLaunchArgument('map_name', default_value='resources/3dmaps/small_forest01cutoff.pcd'),
    ]

    random_forest_node = Node(
        package='map_generator',
        executable='random_forest',
        name='random_forest',
        output='screen',
        parameters=[{
            'init_state_x': LaunchConfiguration('init_x'),
            'init_state_y': LaunchConfiguration('init_y'),
            'map/x_size': LaunchConfiguration('map_size_x_'),
            'map/y_size': LaunchConfiguration('map_size_y_'),
            'map/z_size': LaunchConfiguration('map_size_z_'),
            'map/resolution': 0.1,
            'ObstacleShape/seed': -1,
            'map/obs_num': LaunchConfiguration('p_num'),
            'ObstacleShape/lower_rad': 0.8,
            'ObstacleShape/upper_rad': 1.2,
            'ObstacleShape/lower_hei': 0.8,
            'ObstacleShape/upper_hei': 1.25,
            'map/circle_num': LaunchConfiguration('c_num'),
            'ObstacleShape/radius_l': 0.7,
            'ObstacleShape/radius_h': 0.5,
            'ObstacleShape/z_l': 0.4,
            'ObstacleShape/z_h': 1.0,
            'ObstacleShape/theta': 0.5,
            'sensing/radius': 50.0,
            'sensing/rate': 10.0,
        }],
        remappings=[
            ('~odometry', LaunchConfiguration('odom_topic'))
        ]
    )

    poscmd_2_odom_node = Node(
        package='poscmd_2_odom',
        executable='poscmd_2_odom',
        name='drone_poscmd_2_odom',
        output='screen',
        parameters=[{
            'init_x': LaunchConfiguration('init_x'),
            'init_y': LaunchConfiguration('init_y'),
            'init_z': LaunchConfiguration('init_z'),
        }],
        remappings=[
            ('~command', '/position_cmd'),
            ('~odometry', LaunchConfiguration('odom_topic'))
        ]
    )

    odom_visualization_node = Node(
        package='odom_visualization',
        executable='odom_visualization',
        name='odom_visualization',
        output='screen',
        parameters=[{
            'color/a': 1.0,
            'color/r': 1.0,
            'color/g': 1.0,
            'color/b': 1.0,
            'covariance_scale': 100.0,
            'robot_scale': 1.0,
            'tf45': True
        }],
        remappings=[
            ('~odom', LaunchConfiguration('odom_topic'))
        ]
    )

    pcl_render_node = Node(
        package='local_sensing_node',
        executable='pcl_render_node',
        name='pcl_render_node',
        output='screen',
        parameters=[
            {'sensing_horizon': 50.0,
             'sensing_rate': 30.0,
             'estimation_rate': 30.0,
             'map/x_size': LaunchConfiguration('map_size_x_'),
             'map/y_size': LaunchConfiguration('map_size_y_'),
             'map/z_size': LaunchConfiguration('map_size_z_')}
        ],
        remappings=[
            ('~global_map', '/map_generator/global_cloud'),
            ('~odometry', LaunchConfiguration('odom_topic')),
        ]
    )

    return LaunchDescription(args + [
        random_forest_node,
        poscmd_2_odom_node,
        odom_visualization_node,
        pcl_render_node
    ])
