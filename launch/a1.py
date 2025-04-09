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
            executable='detectObstacle',
            name='detectObstacle',
            output='screen'
        ),
        Node(
            package='turtlebot_pastry',
            executable='followLine',
            name='followLine',
            output='screen'
        ),
    ])