from launch import LaunchDescription
from launch_ros.actions import Node , SetParameter 

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ai_enhanced_minecraft_bringup',
            namespace='actions_api',
            executable='actions_api',
        ),
        Node(
            package='ai_enhanced_minecraft_bringup',
            namespace='image_api',
            executable='image_api',
        ),
        Node(
            package='ai_enhanced_minecraft_tasks',
            executable='generate_datasets',
            parameters=[
                {"image_topic": "/image_api/minecraft_frame"},
                {"actions_topic": "/actions_api/actions"},
                {"image_width": 640},
                {"image_height": 480},
            ],
        )
    ])