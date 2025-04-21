from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_args = [
        DeclareLaunchArgument('map_name', default_value='resources/3dmaps/small_forest01cutoff.pcd'),
        DeclareLaunchArgument('algorithm_name', default_value='thetastar'),
        DeclareLaunchArgument('world_size_x', default_value='40.0'),
        DeclareLaunchArgument('world_size_y', default_value='40.0'),
        DeclareLaunchArgument('world_size_z', default_value='10.0'),
        DeclareLaunchArgument('resolution', default_value='0.1'),
        DeclareLaunchArgument('marker_scale', default_value='0.1'),
        DeclareLaunchArgument('ground_height', default_value='0.01'),
        DeclareLaunchArgument('heuristic', default_value='euclidean'),
        DeclareLaunchArgument('overlay_markers', default_value='false'),
        DeclareLaunchArgument('cost_weight', default_value='4.0'),
        DeclareLaunchArgument('odom_topic', default_value='/visual_slam/odom'),
        DeclareLaunchArgument('camera_pose_topic', default_value='/pcl_render_node/camera_pose'),
        DeclareLaunchArgument('depth_topic', default_value='/pcl_render_node/depth'),
        DeclareLaunchArgument('cloud_topic', default_value='/map_generator/global_cloud'),
        DeclareLaunchArgument('cx', default_value='321.04638671875'),
        DeclareLaunchArgument('cy', default_value='243.44969177246094'),
        DeclareLaunchArgument('fx', default_value='387.229248046875'),
        DeclareLaunchArgument('fy', default_value='387.229248046875'),
        DeclareLaunchArgument('init_x', default_value='-5.0'),
        DeclareLaunchArgument('init_y', default_value='0.0'),
        DeclareLaunchArgument('init_z', default_value='0.1'),
        DeclareLaunchArgument('output', default_value='screen')
    ]

    # Main planner node
    planner_node = Node(
        package='heuristic_planners',
        executable='planner_ros_node',
        name='global_planner',
        output=LaunchConfiguration('output'),
        # prefix='gdb -ex run --args',
        parameters=[{
            'resolution': LaunchConfiguration('resolution'),
            'frame_id': 'world',
            'overlay_markers': LaunchConfiguration('overlay_markers'),
            'algorithm': LaunchConfiguration('algorithm_name'),
            'cost_weight': LaunchConfiguration('cost_weight'),
            'heuristic': LaunchConfiguration('heuristic'),
            'marker_scale': LaunchConfiguration('marker_scale'),
            'ground_height': LaunchConfiguration('ground_height'),
            'init_x': LaunchConfiguration('init_x'),
            'init_y': LaunchConfiguration('init_y'),
            'init_z': LaunchConfiguration('init_z'),

            # algorithm-specific parameters
            'astar.resolution': 0.1,
            'astar.time_resolution': 0.8,
            'astar.lambda_heuristic': 5.0,
            'astar.allocate_num': 1000000,

            'thetastar.resolution': 0.1,
            'thetastar.time_resolution': 0.8,
            'thetastar.lambda_heuristic': 5.0,
            'thetastar.allocate_num': 1000000,

            'thetastaragr.resolution': 0.1,
            'thetastaragr.time_resolution': 0.8,
            'thetastaragr.allocate_num': 1000000,
            'thetastaragr.lambda_heuristic': 5.0,
            'thetastaragr.ground_judge': 0.1,
            'thetastaragr.barrier': 2.0,
            'thetastaragr.flying_cost': 10.0,
            'thetastaragr.flying_cost_default': 0,
            'thetastaragr.epsilon': 0.1,

            # grid_map params
            'grid_map.resolution': 0.1,
            'grid_map.map_size_x': LaunchConfiguration('world_size_x'),
            'grid_map.map_size_y': LaunchConfiguration('world_size_y'),
            'grid_map.map_size_z': LaunchConfiguration('world_size_z'),
            'grid_map.local_update_range_x': 40.0,
            'grid_map.local_update_range_y': 40.0,
            'grid_map.local_update_range_z': 10.0,
            'grid_map.obstacles_inflation': 0.099,
            'grid_map.local_map_margin': 30,
            'grid_map.ground_height': LaunchConfiguration('ground_height'),
            'grid_map.cx': LaunchConfiguration('cx'),
            'grid_map.cy': LaunchConfiguration('cy'),
            'grid_map.fx': LaunchConfiguration('fx'),
            'grid_map.fy': LaunchConfiguration('fy'),
            'grid_map.use_depth_filter': True,
            'grid_map.depth_filter_tolerance': 0.15,
            'grid_map.depth_filter_maxdist': 5.0,
            'grid_map.depth_filter_mindist': 0.2,
            'grid_map.depth_filter_margin': 1,
            'grid_map.k_depth_scaling_factor': 1000.0,
            'grid_map.skip_pixel': 2,
            'grid_map.p_hit': 0.65,
            'grid_map.p_miss': 0.35,
            'grid_map.p_min': 0.12,
            'grid_map.p_max': 0.90,
            'grid_map.p_occ': 0.80,
            'grid_map.min_ray_length': 0.1,
            'grid_map.max_ray_length': 4.5,
            'grid_map.virtual_ceil_height': 2.5,
            'grid_map.visualization_truncate_height': 2.4,
            'grid_map.show_occ_time': False,
            'grid_map.pose_type': 2,
            'grid_map.frame_id': 'world'
        }],
        remappings=[
            ('/grid_map/cloud', '/pcl_render_node/cloud'),
            ('/grid_map/odom', LaunchConfiguration('odom_topic')),
            ('/grid_map/pose', LaunchConfiguration('camera_pose_topic')),
            ('/grid_map/depth', LaunchConfiguration('depth_topic')),
            ('goal', '/move_base_simple/goal')
        ]
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('heuristic_planners'),
            'rviz',
            'planners.rviz'
        ])],
        output='screen'
    )


    # simulator_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([
    #             FindPackageShare('heuristic_planners'),
    #             'launch',
    #             'simulator_launch.py'
    #         ])
    #     ),
    #     launch_arguments={
    #         'map_name': LaunchConfiguration('map_name'),
    #         'map_size_x_': LaunchConfiguration('world_size_x'),
    #         'map_size_y_': LaunchConfiguration('world_size_y'),
    #         'map_size_z_': LaunchConfiguration('world_size_z'),
    #         'init_x': LaunchConfiguration('init_x'),
    #         'init_y': LaunchConfiguration('init_y'),
    #         'init_z': LaunchConfiguration('init_z'),
    #         'c_num': '0',
    #         'p_num': '80',
    #         'odom_topic': LaunchConfiguration('odom_topic')
    #     }.items()
    # )

    return LaunchDescription(declared_args + [planner_node, rviz_node])
