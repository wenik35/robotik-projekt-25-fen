from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot_pastry',
            executable='stateMachine',
            name='stateMachine',
            output='screen',
            arguments=['--ros-args', '--log-level', 'INFO']
        ),
        Node(
            package='turtlebot_pastry',
            executable='detectObstacle',
            name='detectObstacle',
            output='screen',
            arguments=['--ros-args', '--log-level', 'INFO']
        ),
        Node(
            package='turtlebot_pastry',
            executable='followPath',
            name='followPath',
            output='screen',
            arguments=['--ros-args', '--log-level', 'INFO']
        ),
        Node(
            package='turtlebot_pastry',
            executable='signRecognition',
            name='signRecognition',
            output='screen',
            arguments=['--ros-args', '--log-level', 'INFO']
        ),
    ])
