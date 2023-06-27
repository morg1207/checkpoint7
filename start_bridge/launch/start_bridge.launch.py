from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros1_bridge',
            executable='action_bridge',
            namespace="bridge1",
            arguments=["ros1", "control_msgs", "FollowJointTrajectory", "/scaled_pos_joint_traj_controller/follow_joint_trajectory"],
        ),
        Node(
            package='ros1_bridge',
            executable='action_bridge',
            namespace="bridge2",
            arguments=["ros1", "control_msgs", "GripperCommand", "/gripper_controller/gripper_cmd"],
        ),
    ])