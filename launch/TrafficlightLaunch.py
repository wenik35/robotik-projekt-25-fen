from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot_pastry',
            executable='trafficlight_start',
            name='trafficlight_start',
            output='screen'
        ),
        Node(
            package='turtlebot_pastry',
            executable='stateMachine',
            name='stateMachineNode',
            output='screen'
        ),
        Node(
            package='turtlebot_pastry',
            executable='changeLaneAtObstacle',
            name='changeLaneAtObstacleNode',
            output='screen'
        ),
        Node(
            package='turtlebot_pastry',
            executable='followPath',
            name='followPathNode',
            output='screen'
        ),
        Node(
            package='turtlebot_pastry',
            executable='signRecognition',
            name='signRecognition',
            output='screen',
            #arguments=['--ros-args', '--log-level', 'INFO']
        ),
        Node(
            package='turtlebot_pastry',
            executable='parking',
            name='parking',
            output='screen',
            arguments=['--ros-args', '--log-level', 'INFO']
        ),
        Node(
            package='turtlebot_pastry',
            executable='crossing',
            name='crossing',
            output='screen',
            arguments=['--ros-args', '--log-level', 'INFO']
           ),
    ])
