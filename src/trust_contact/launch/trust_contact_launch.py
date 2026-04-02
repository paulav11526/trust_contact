from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = 'trust_contact'
    return LaunchDescription([
        # Mujoco simulator
        Node(
            package=package_name,
            executable='simulation',
            name='mujoco_simulator',
            output='screen'
        ),
        #FSM node
        Node(
            package=package_name,
            executable='fsm_node',
            name='fsm_logic',
            output='screen'
        ),
        # Momentum observer
        Node(
            package=package_name,
            executable='residual_node',
            name='momentum_observer',
            output='screen'
        ),

        # Classifier
        Node(
            package=package_name,
            executable='contact_classifier_node',
            name='RF_classifier',
            output='screen'
        )

        
    ])