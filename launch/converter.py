from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare the launch argument
        DeclareLaunchArgument(
            'quality',
            default_value='80',
            description='JPEG quality setting for image compression'
        ),
        
        # Define the node
        Node(
            package='image_compressed',
            executable='image_compressed',
            name='image_compressed',
            output='screen',
            parameters=[{'quality': LaunchConfiguration('quality')}]
        ),
    ])
