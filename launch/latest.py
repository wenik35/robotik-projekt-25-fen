from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot_pastry',
            executable='stateMachine',
            name='stateMachine',
            output='screen'
        ),
        Node(
            package='turtlebot_pastry',
            executable='changeLaneAtObstacle',
            name='changeLaneAtObstacle',
            output='screen'
        ),
        Node(
            package='turtlebot_pastry',
            executable='imageProcessing',
            name='imageProcessing',
            output='screen'
        ),
        Node(
            package='turtlebot_pastry',
            executable='followPath',
            name='followPath',
            output='screen'
        ),
        Node(
            package='turtlebot_pastry',
            executable='signRecognition',
            name='signRecognition',
            output='screen',
            arguments=['--ros-args', '--log-level', 'INFO']
        ),
        Node(
            package='turtlebot_pastry',
            executable='parking',
            name='parking',
            output='screen',
            arguments=['--ros-args', '--log-level', 'INFO']
        ),
    ])
