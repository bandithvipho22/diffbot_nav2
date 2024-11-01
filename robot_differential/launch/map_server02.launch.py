import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    saved_map_path = '/home/vipho/dev_ws/src/robot_differential/map/saved_map02' 
    yaml_filename = saved_map_path + '.yaml'
    
    # Launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    
    # Define lifecycle nodes for navigation stack
    lifecycle_nodes = ['map_server', 'amcl']

    # Parameter substitutions for use_sim_time and map yaml file
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': yaml_filename}

    # Rewriting YAML configuration with substitutions
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    # Save map command
    save_map_cmd = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', saved_map_path],
        output='screen',
        shell=True
    )

    # Wait for map to be saved
    wait_for_map_cmd = TimerAction(
        period=10.0,  
        actions=[]
    )

    # Launch Description for the entire navigation stack
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                get_package_share_directory('robot_differential'),
                'config',
                'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use'),

        # Save the map from SLAM
        save_map_cmd,

        # Wait for the map to be saved
        wait_for_map_cmd,

    
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'yaml_filename': yaml_filename}]),

        # Node to start AMCL for localization
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[configured_params],
            remappings=[('/tf', 'tf'),
                        ('/tf_static', 'tf_static')]),

        # Node to manage the lifecycle of navigation nodes
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}]),

        # (Optional) Static transform publisher for map to odom frame
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=['0.0', '0', '0.0', '0', '0', '0', 'map', 'odom'],
        #     output='screen'
        # ),
    ])
