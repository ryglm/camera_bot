from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop',
            executable='teleop',
            name='teleop',
            output='screen',
            parameters=[{
                'steering_topic': '/ugv/steering_controller/commands',
                'rear_topic': '/ugv/rear_wheels_controller/commands',
                'max_steer_rad': 0.6,
                'max_speed_radps': 12.0,
                'step_steer': 0.02,
                'step_speed': 0.5,
                'publish_rate_hz': 30.0,
                'auto_center': True,
                'center_decay_per_sec': 1.5,
                'deadman_timeout_sec': 0.7,
            }],
        ),
    ])
