# launch/sim_with_rviz.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro  # xacro가 필요하면 ros-$ROS_DISTRO-xacro 설치

def generate_launch_description():
    pkg_share = get_package_share_directory('tutorial_ros2_motor')
    assets = os.path.join(pkg_share, 'assets')

    mjcf = os.path.join(assets, 'mobile_robot.xml')
    urdf_xacro = os.path.join(assets, 'mobile_robot.urdf.xacro')
    rviz_cfg = os.path.join(assets, 'mobile_robot.rviz')

    # .xacro -> XML로 변환
    robot_desc = xacro.process_file(urdf_xacro).toxml()

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen'
    )

    sim = Node(
        package='tutorial_ros2_motor',
        executable='mujoco_sim_node',
        parameters=[{'mjcf_path': mjcf}],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen'
    )

    return LaunchDescription([rsp, sim, rviz])
