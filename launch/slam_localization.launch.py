import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    slam_pkg = get_package_share_directory('slam_toolbox')
    this_pkg = get_package_share_directory('lidar_2d_mapping')

    localization_params = os.path.join(this_pkg, 'params', 'mapper_params_localization.yaml')
  
    set_rmw = SetEnvironmentVariable(
        name='RMW_IMPLEMENTATION',
        value='rmw_cyclonedds_cpp'
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_pkg, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'slam_params_file': localization_params,
            'use_sim_time': 'true'
        }.items()
    )

    rviz_config_path = os.path.join(this_pkg, 'rviz', 'slam_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        set_rmw,
        slam_toolbox_launch,
        rviz_node
    ])
