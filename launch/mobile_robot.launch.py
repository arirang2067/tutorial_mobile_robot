from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('tutorial_mobile_robot')
    assets    = PathJoinSubstitution([pkg_share, 'assets'])
    rviz_cfg  = PathJoinSubstitution([assets, 'mobile_robot.rviz'])

    mobile_node = Node(
        package="tutorial_mobile_robot",
        executable="mobile_robot_node",
        name="mobile_robot_node",
        output="screen",
        parameters=[{
            "loop_hz": 100.0,
            "publish_tf": True,
            "base_frame_id": "base_link",
            "odom_frame_id": "odom",
            "left_joint_name": "wheel_left_joint",
            "right_joint_name": "wheel_right_joint",
            "wheel_radius": 0.065,
            "wheel_length": 0.4465,
            "gear_ratio": 1.0,
            "max_rpm": 400,
            "use_rate_limit": False,
            "bound_cmd_speed": 0.15,
            "add_cmd_speed": 0.03,
            "bound_cmd_ang_speed": 0.30,
            "add_cmd_ang_speed": 0.06,
            "deadzone_linear": 0.03,
            "deadzone_angular": 0.03,
            "lowpass_alpha_linear": 0.0,
            "lowpass_alpha_angular": 0.0
        }]
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
