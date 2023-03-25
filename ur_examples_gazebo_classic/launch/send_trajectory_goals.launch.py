
# Author: Dan Zimmerman
#
# Description: Send a series of test postures to the running Gazebo simulation.

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    posture_goals = PathJoinSubstitution(
        [FindPackageShare("ur_examples_gazebo_classic"), "config", "test_goals_config.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="ros2_controllers_test_nodes",
                executable="publisher_joint_trajectory_controller",
                name="publisher_joint_trajectory_controller",
                parameters=[posture_goals],
                output="screen",
            )
        ]
    )
