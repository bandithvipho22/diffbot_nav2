import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
from launch_ros.actions import Node
import os

def generate_launch_description():
    initial_pose_x = 0
    initial_pose_y = 0
    initial_yaw = 0
    pkg_share = launch_ros.substitutions.FindPackageShare(package='robot_differential').find('robot_differential')
    default_model_path = os.path.join(pkg_share, 'urdf/robot_differential.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/robot.rviz')
    # default_model_path = os.path.join(pkg_share, 'urdf/turtlebot3_burger/turtlebot3_burger.urdf')
    # default_model_path = os.path.join(pkg_share, 'urdf/amr_autobot/AMR.xacro')
    #-------------------------------------------------------
    # world_file_name = 'turtlebot3_hourse/' + '.model'
    world_path=os.path.join(pkg_share, 'world/bongworld.sdf') #yolo_world.sdf, bongworld.sdf
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
        # parameters=LaunchConfiguration('model')
    )
    # Create a robot_state_publisher node
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
        arguments=['-entity', 'robot_differential', '-topic', 'robot_description'],
        output='screen'
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='false',
                                            description='Use sim time if true'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.ExecuteProcess(cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
        #gazebo
        # Node for wheel 
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=['0.0', '0.1485', '0.0', '-1.57', '0', '0', 'base_link', 'left_wheel'],
        #     output='screen'
        # ),
        
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=['0.0', '-0.1485', '0', '1.57', '0', '0', 'base_link', 'right_wheel'],
        #     output='screen'
        # ),
        # joint_state_publisher_node,
        #----------------------------------------------------------------------------
        launch_ros.actions.Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments = [str(initial_pose_x), str(initial_pose_y), "0", str(initial_yaw), "0", "0", "map", "odom"]
        ),
        #-----------------------------------------------------------------------------
        node_robot_state_publisher,
        robot_state_publisher_node,
        # joint_state_publisher_gui_node,
        spawn_entity
    ])
