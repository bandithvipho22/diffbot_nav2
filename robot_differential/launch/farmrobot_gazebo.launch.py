import launch
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
from launch_ros.actions import Node

import os

AIFARM_MODEL = os.environ['AIFARM_MODEL']


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='farmrobot_description').find('farmrobot_description')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/robot.rviz')

    use_model=AIFARM_MODEL +'.urdf'
    
    if use_model =='turtlebot3_burger.urdf':
        print("Welcome, It's turtlebot3_burger")
        model_path = os.path.join(pkg_share,
                         'urdf/turtlebot3_burger', use_model)
        
    elif use_model == 'robot_differential.urdf':
        print("Welcome, It's robot_differential")
        model_path = os.path.join(pkg_share,
                         'urdf/robot_differential' ,use_model)
    #-------------------------------------------------------
    world_path=os.path.join(pkg_share, 'world/world12.sdf')
    use_sim_time = LaunchConfiguration('use_sim_time')
    model = LaunchConfiguration('model')
    #-------------------------------------------------------
    params = {'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    
    spawn_entity = launch_ros.actions.Node(
    	package='gazebo_ros', 
    	executable='spawn_entity.py',
        arguments=['-entity', 'robot_differential', '-topic', 'robot_description','-x', '1'],
        output='screen'
    )

    start_joint_state_publisher_cmd = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    remappings=[("/robot_description", "/unicycle_bot_robot_description")]
    )
    return launch.LaunchDescription([ 
        
        launch.actions.DeclareLaunchArgument(name='gui', default_value='false',
                                            description='Use sim time if true'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
        node_robot_state_publisher,
        start_joint_state_publisher_cmd ,
        Node(
                # condition=IfCondition(use_robot_state_pub),
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[{'use_sim_time': use_sim_time, 
                'robot_description': Command(['xacro ', model])}],
                arguments=[use_model]),
        spawn_entity
    ])