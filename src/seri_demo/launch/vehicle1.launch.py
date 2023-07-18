# pull in some Python launch modules.
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()

    # Loading parameters from a file
    node_params = os.path.join(
        get_package_share_directory('seri_demo'),
        'config',
        'params.yaml'
    )
    vehicle1_node = Node(
        package="seri_demo",
        executable="vehicle1.py",
        parameters=[node_params],
    )

    ld.add_action(vehicle1_node)
    return ld
