# pull in some Python launch modules.
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
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
    # vehicle1_node = Node(
    #     package="seri_demo",
    #     executable="vehicle.py",
    #     name="vehicle1_commander",
    #     parameters=[node_params],
    # )

    gotogoal_v1 = ExecuteProcess(
        cmd=[[
            f'ROS_DOMAIN_ID=1 ros2 run seri_demo vehicle.py --ros-args -r __node:=vehicle1_commander --params-file {node_params}',
        ]],
        shell=True
    )
    
    gotogoal_v2 = ExecuteProcess(
        cmd=[[
            f'ROS_DOMAIN_ID=2 ros2 run seri_demo vehicle.py --ros-args -r __node:=vehicle2_commander --params-file {node_params}',
        ]],
        shell=True
    )
    
    gotogoal_v3 = ExecuteProcess(
        cmd=[[
            f'ROS_DOMAIN_ID=3 ros2 run seri_demo vehicle.py --ros-args -r __node:=vehicle3_commander --params-file {node_params}',
        ]],
        shell=True
    )

    ld.add_action(gotogoal_v1)
    return ld
