from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable, TextSubstitution
import yaml

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=_setup)
    ])

def _setup(context, *args, **kwargs):
    pkg_share   = FindPackageShare('tutorial_mobile_robot')
    assets      = PathJoinSubstitution([pkg_share, 'assets'])
    urdf_xacro  = PathJoinSubstitution([assets, 'mobile_robot.urdf.xacro'])
    mjcf        = PathJoinSubstitution([assets, 'mobile_robot.xml'])
    params_yaml = PathJoinSubstitution([assets, 'params.yaml'])
    use_sim_time = LaunchConfiguration('use_sim_time')

    params_path = params_yaml.perform(context)
    with open(params_path, 'r') as f:
        y = yaml.safe_load(f) or {}
    sim_params = (y.get('mujoco_sim_node') or {}).get('ros__parameters') or {}

    xacro_exec = FindExecutable(name='xacro')
    robot_description = ParameterValue(
        Command([xacro_exec, TextSubstitution(text=' '), urdf_xacro]),
        value_type=str
    )

    # 3) 노드들
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
        output='screen'
    )

    desc_pub = Node(
        package='tutorial_mobile_robot',
        executable='robot_description_topic_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    sim = Node(
        package='tutorial_mobile_robot',
        executable='mujoco_sim_node',
        name='mujoco_sim_node',
        output='screen',
        parameters=[
            sim_params,
            {'use_sim_time': use_sim_time, 'mjcf_path': mjcf}
        ],
    )

    return [jsp, rsp, desc_pub, sim]
