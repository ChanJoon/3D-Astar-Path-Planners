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
        name='map_generator_node',
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
            ('odometry', LaunchConfiguration('odom_topic'))
        ]
    )



    return LaunchDescription(args + [
        random_forest_node,
    ])
