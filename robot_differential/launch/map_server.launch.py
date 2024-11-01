# Copyright (c) 2018 Intel Corporation
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml



def generate_launch_description():

    #############################################
    # map_file=os.path.join(get_package_share_directory('ros_foxy'),'config','map.yaml')
    map_dir = LaunchConfiguration(
        'maps',
        default=os.path.join(
            get_package_share_directory('robot_differential'),
            'map',
            'map_flood4x15.yaml'))  #  my_map_save.yaml


    lifecycle_nodes = ['map_server']

    return LaunchDescription([
        DeclareLaunchArgument(
            'maps',
            default_value=map_dir,
            description='Full path to map file to load'),


        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'yaml_filename':map_dir}]),
                        
        # Node(
        # package='nav2_amcl',
        # executable='amcl',
        # name='amcl',
        # output='screen',
        # # parameters=[nav2_yaml]
        # ),

        Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mapper',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': lifecycle_nodes}]
        ),
        
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=['0.0', '0', '0.0', '0', '0', '0', 'map', 'odom'],
        #     output='screen'
        # ),
    ])

# import os

# from ament_index_python.packages import get_package_share_directory

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node
# from nav2_common.launch import RewrittenYaml

# def generate_launch_description():
#     # Define the map file path
#     map_dir = LaunchConfiguration(
#         'map',
#         default=os.path.join(
#             get_package_share_directory('robot_differential'),
#             'map',
#             'astar_map.yaml'))

#     # Launch configuration variables
#     namespace = LaunchConfiguration('namespace')
#     use_sim_time = LaunchConfiguration('use_sim_time')
#     autostart = LaunchConfiguration('autostart')
#     params_file = LaunchConfiguration('params_file')
    
#     # Define lifecycle nodes for navigation stack
#     lifecycle_nodes = ['map_server', 'amcl']

#     # Parameter substitutions for use_sim_time and map yaml file
#     param_substitutions = {
#         'use_sim_time': use_sim_time,
#         'yaml_filename': map_dir}

#     # Rewriting YAML configuration with substitutions
#     configured_params = RewrittenYaml(
#         source_file=params_file,
#         root_key=namespace,
#         param_rewrites=param_substitutions,
#         convert_types=True)

#     # Launch Description for the entire navigation stack
#     return LaunchDescription([
#         # Declare arguments
#         DeclareLaunchArgument(
#             'namespace', default_value='',
#             description='Top-level namespace'),

#         DeclareLaunchArgument(
#             'map',
#             default_value=map_dir,
#             description='Full path to map yaml file to load'),

#         DeclareLaunchArgument(
#             'use_sim_time', default_value='true',
#             description='Use simulation (Gazebo) clock if true'),

#         DeclareLaunchArgument(
#             'autostart', default_value='true',
#             description='Automatically startup the nav2 stack'),

#         DeclareLaunchArgument(
#             'params_file',
#             default_value=os.path.join(
#                 get_package_share_directory('robot_differential'),
#                 'config',
#                 'nav2_params.yaml'),
#             description='Full path to the ROS2 parameters file to use'),

#         # Node to start the map server
#         Node(
#             package='nav2_map_server',
#             executable='map_server',
#             name='map_server',
#             output='screen',
#             parameters=[{'use_sim_time': use_sim_time},
#                         {'yaml_filename': map_dir}]),

#         # Node to start AMCL for localization
#         Node(
#             package='nav2_amcl',
#             executable='amcl',
#             name='amcl',
#             output='screen',
#             parameters=[configured_params],
#             remappings=[('/tf', 'tf'),
#                         ('/tf_static', 'tf_static')]),

#         # Node to manage the lifecycle of navigation nodes
#         Node(
#             package='nav2_lifecycle_manager',
#             executable='lifecycle_manager',
#             name='lifecycle_manager_localization',
#             output='screen',
#             parameters=[{'use_sim_time': use_sim_time},
#                         {'autostart': autostart},
#                         {'node_names': lifecycle_nodes}]),

#         # (Optional) Static transform publisher for map to odom frame
#         # Node(
#         #     package='tf2_ros',
#         #     executable='static_transform_publisher',
#         #     arguments=['0.0', '0', '0.0', '0', '0', '0', 'map', 'odom'],
#         #     output='screen'
#         # ),
#     ])
