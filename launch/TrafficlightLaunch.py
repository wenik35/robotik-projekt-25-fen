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
            executable='state_machine',
            name='state_machine',
            output='screen'
        ),
        Node(
            package='turtlebot_pastry',
            executable='change_lane',
            name='change_lane',
            output='screen'
        ),
        Node(
            package='turtlebot_pastry',
            executable='follow_path',
            name='follow_path',
            output='screen'
        ),
        Node(
            package='turtlebot_pastry',
            executable='sign_recognition',
            name='sign_recognition',
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
