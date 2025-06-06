
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tennis_ball_tracker',
            executable='ball_detector_node', # Name of the entry point in setup.py
            name='ball_detector', # Name of the node when it runs
            output='screen',
            emulate_tty=True,
            parameters=[
                {'image_topic': '/image_raw'}, # camera's image topic
               
                {'min_radius': 10} # Minimum radius of the ball to be considered detected
            ]
        )
      
    ])
