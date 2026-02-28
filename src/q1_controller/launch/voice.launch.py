from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='q1_controller',
            executable='gamepad_publisher.py',
            name='gamepad_publisher',
            output='screen',
        ),
        Node(
            package='q1_controller',
            executable='tts_say.py',
            name='aiui_tts_node',
            output='screen',
        ),
        Node(
            package='q1_controller',
            executable='voice_trigger.py',
            name='voice_trigger',
            output='screen',
        ),
    ])