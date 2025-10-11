# launch/serial_bridge.launch.py
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import yaml

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_launch_setup)])

def _launch_setup(context, *args, **kwargs):
    pkg_share   = FindPackageShare('tutorial_mobile_robot')
    assets      = PathJoinSubstitution([pkg_share, 'assets'])
    params_yaml = PathJoinSubstitution([assets, 'params.yaml'])

    params_path = params_yaml.perform(context)
    with open(params_path, 'r') as f:
        y = yaml.safe_load(f) or {}

    serial_params = (y.get('serial_bridge_node') or {}).get('ros__parameters') or {}
    if not serial_params:
        raise RuntimeError(
            f"[serial_bridge.launch] assets/params.yaml에 "
            f"'serial_bridge_node: {{ ros__parameters: ... }}' 블록이 비어있거나 없습니다. 경로: {params_path}"
        )

    serial_bridge = Node(
        package='tutorial_mobile_robot',
        executable='serial_bridge_node',
        name='serial_bridge_node',
        output='screen',
        parameters=[serial_params],
    )

    return [serial_bridge]
