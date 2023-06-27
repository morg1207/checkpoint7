from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='simple_grasping',
            executable='basic_grasping_perception_node',
            name='server_detection_object'
        ),
        Node(
            package='get_pose_perception',
            executable='get_pose_client',
            name='get_pose_client'
        ),
    ])