from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('tutorial_mobile_robot')
    assets    = PathJoinSubstitution([pkg_share, 'assets'])
    rviz_cfg  = PathJoinSubstitution([assets, 'mobile_robot.rviz'])
    params_yaml = PathJoinSubstitution([assets, 'params.yaml'])

    mobile_node = Node(
        package="tutorial_mobile_robot",
        executable="mobile_robot_node",
        name="mobile_robot_node",
        output="screen",
        parameters=[params_yaml],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_cfg],
        output='screen'
    )

    return LaunchDescription([
        mobile_node,
        rviz
    ])
