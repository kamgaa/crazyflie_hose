from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Leader node 실행
        Node(
            package='cmd',
            executable='leader_node',
            name='leader_node',
            output='screen'
        ),

        # Follower node 실행
        Node(
            package='cmd',
            executable='follower_node',
            name='follower_node',
            output='screen'
        ),
        Node(
            package='cmd',
            executable='end_node',
            name='end_node',
            output='screen'
        ),

        Node(
            package='cmd',
            executable='hover_sync_node',
            name='hover_sync_node',
            output='screen'
        ),

    ])

