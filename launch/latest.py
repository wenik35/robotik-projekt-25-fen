from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
            executable='parking',
            name='parkingNode',
            output='screen'
        ),
        Node(
            package='turtlebot_pastry',
            executable='signRecognition',
            name='signRecognitionNode',
            output='screen'
        ),
        Node(
            package='turtlebot_pastry',
            executable='trafficlight_start',
            name='trafficlight_startNode',
            output='screen'
        ),
    ])

'''


'''