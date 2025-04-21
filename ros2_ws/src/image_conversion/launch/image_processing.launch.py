from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Arguments for image_conversion node
        DeclareLaunchArgument(
            'input_topic',
            default_value='/image_raw',
            description='Input topic for image conversion node'
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/converted_image',
            description='Output topic for image conversion node'
        ),
        
        # Camera Publisher Node
        Node(
            package='camera_publisher',
            executable='camera_publisher',
            name='camera_publisher',
            output='screen',
            parameters=[{
                'publish_rate': 30.0, 
            }]
        ),
        
        # Image Conversion Node
        Node(
            package='image_conversion',
            executable='image_conversion_node',
            name='image_conversion_node',
            output='screen',
            parameters=[{
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
            }]
        ),
        
    ])
