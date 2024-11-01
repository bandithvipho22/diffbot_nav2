from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'tf_buffer_duration',
            default_value='60.0',
            description='Duration to buffer TF data'),
        
        DeclareLaunchArgument(
            'publish_frequency',
            default_value='10.0',
            description='Frequency to publish TF data'),
        
        DeclareLaunchArgument(
            'transform_tolerance',
            default_value='1.0',
            description='How long to wait for transform lookup'),
        
        
        Node(
            package='robot_differential',  
            executable='pose_pub.py',
            name='initial_pose_setter',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'tf_buffer_duration': LaunchConfiguration('tf_buffer_duration'),
                'publish_frequency': LaunchConfiguration('publish_frequency'),
                'transform_tolerance': LaunchConfiguration('transform_tolerance'),
            }]
        ),
        
        # Add other nodes here as needed, such as:
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_tf_pub_map_odom',
        #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        # ),
    ])