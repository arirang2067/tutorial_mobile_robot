# launch/mobile_robot.launch.py
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml  # 없으면: sudo apt-get install -y python3-yaml

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=_launch_setup)
    ])

def _launch_setup(context, *args, **kwargs):
    pkg_share   = FindPackageShare('tutorial_mobile_robot')
    assets      = PathJoinSubstitution([pkg_share, 'assets'])
    rviz_cfg    = PathJoinSubstitution([assets, 'mobile_robot.rviz'])
    params_yaml = PathJoinSubstitution([assets, 'params.yaml'])

    params_path = params_yaml.perform(context)
    with open(params_path, 'r') as f:
        y = yaml.safe_load(f) or {}

    mobile_params = (y.get('mobile_robot_node') or {}).get('ros__parameters') or {}
    if not mobile_params:
        raise RuntimeError(f"[mobile_robot.launch] '{params_path}'에 "
                           f"mobile_robot_node.ros__parameters가 없습니다.")

    mobile_node = Node(
        package="tutorial_mobile_robot",
        executable="mobile_robot_node",
        name="mobile_robot_node",
        output="screen",
        parameters=[mobile_params],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_cfg],
        output='screen'
    )

    return [mobile_node, rviz]
