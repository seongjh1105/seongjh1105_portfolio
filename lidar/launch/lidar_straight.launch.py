#!/usr/bin/env python3

import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    main_param_dir = launch.substitutions.LaunchConfiguration(
        'main_param_dir',
        default=os.path.join(
            get_package_share_directory('lidar'),
            'config',
            'straight_parameters.yaml'))

    rviz_param_dir = launch.substitutions.LaunchConfiguration(
        'rviz_param_dir',
        default=os.path.join(
            get_package_share_directory('lidar'),
            'config',
            'lidar_straight_rviz2_config.rviz'))

    sampling = launch_ros.actions.Node(
        package='lidar',
        executable='sampling',
        name='sampling',
        parameters=[main_param_dir]
    )

    roi_straight = launch_ros.actions.Node(
        package='lidar',
        executable='roi_straight',
        name='roi_straight',
        parameters=[main_param_dir]
    )

    ransac = launch_ros.actions.Node(
        package='lidar',
        executable='ransac',
        name='ransac',
        parameters=[main_param_dir]
    )

    dbscan = launch_ros.actions.Node(
        package='lidar',
        executable='dbscan',
        name='dbscan',
        parameters=[main_param_dir]
    )

    rviz = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_param_dir]
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'main_param_dir',
            default_value=main_param_dir,
            description='Full path to main parameter file to load'),
        launch.actions.DeclareLaunchArgument(
            'rviz_param_dir',
            default_value=rviz_param_dir,
            description='Full path to RViz configuration file to load'),
        sampling,
        roi_straight,
        ransac,
        dbscan,
        rviz,
    ])
