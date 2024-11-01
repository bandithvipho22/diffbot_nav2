
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Include localization, navigation and map (rviz) in 1-file
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('robot_differential'),
            'map','new_map.yaml')) #, 'new_map.yaml'
           

    # param_dir = LaunchConfiguration(
    #     'params_file',
    #     default=os.path.join(
    #         get_package_share_directory('robot_differential'),
    #         'urdf/robot_differential.urdf'))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('robot_differential'), 
                                        'launch', 'robot_nav2.launch.py') #'/robot_nav2.launch.py'

    rviz_config_dir = os.path.join(
        get_package_share_directory('robot_differential'),
        'rviz',
        'new_robot3.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        # DeclareLaunchArgument(
        #     'params_file',
        #     default_value=param_dir,
        #     description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir]),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                }.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
