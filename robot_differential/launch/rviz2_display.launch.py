import os
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
import launch_ros
import launch

def generate_launch_description():
    
    # pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_share = launch_ros.substitutions.FindPackageShare(package='robot_differential').find('robot_differential')
    
    default_model_path = os.path.join(pkg_share, 'urdf/robot_differential.urdf')
    rviz_config_dir = os.path.join(
        get_package_share_directory('robot_differential'),
        'rviz') #'robot.rviz'
    
    # use_sim_time = LaunchConfiguration('use_sim_time')
    
    
    # spawn_entity = launch_ros.actions.Node(
    # 	package='gazebo_ros', 
    # 	executable='spawn_entity.py',
    #     arguments=['-entity', 'robot_differential', '-topic', 'robot_description'],
    #     output='screen'
    # )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        # condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
        # parameters=LaunchConfiguration('model')
    )
    # Create a robot_state_publisher node
    # params = {'use_sim_time': use_sim_time}
    # node_robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=[params]
    # )
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='false',
                                            description='Use sim time if true'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        
         DeclareLaunchArgument(
    name='use_simulator',
    default_value='True',
    description='Whether to start the simulator'),
        DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Whether to execute gzclient'),

    # # Start Gazebo server
    #     IncludeLaunchDescription(
    # PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    # # condition=IfCondition(use_simulator),
    # # launch_arguments={'world': world_path}.items()
    # ),

    # Start Gazebo client    
    #     IncludeLaunchDescription(
    # PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    # condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless]))
    # ),
    
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['-0.090', '0', '0.112', '0', '0', '0', 'chassis', 'laser_frame'],
            output='screen'
        ),
         Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            output='screen'
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['-0.170', '0', '0.01', '0', '0', '0', 'base_link', 'chassis'],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
            output='screen'
        ),
       
       
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'),
        # node_robot_state_publisher,
        robot_state_publisher_node,
        joint_state_publisher_node,
        # spawn_entity,
        
    ])
