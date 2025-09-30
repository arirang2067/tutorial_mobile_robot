from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share   = FindPackageShare('tutorial_mobile_robot')
    assets      = PathJoinSubstitution([pkg_share, 'assets'])
    urdf_xacro  = PathJoinSubstitution([assets, 'mobile_robot.urdf.xacro'])
    mjcf        = PathJoinSubstitution([assets, 'mobile_robot.xml'])
    rviz_cfg    = PathJoinSubstitution([assets, 'mobile_robot.rviz'])

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    publish_tf_arg   = DeclareLaunchArgument('publish_tf',   default_value='true')
    loop_hz_arg      = DeclareLaunchArgument('loop_hz',      default_value='200.0')

    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_tf   = LaunchConfiguration('publish_tf')
    loop_hz      = LaunchConfiguration('loop_hz')

    xacro_exec = FindExecutable(name='xacro')
    robot_description = ParameterValue(
        Command([xacro_exec, TextSubstitution(text=' '), urdf_xacro]),
        value_type=str
    )

    # joint_state_publisher (있으면 유지)
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # robot_state_publisher (TF 생성용)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # odom -> base_footprint 정적 TF (원하면 유지)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
        '--x','0','--y','0','--z','0',
        '--qx','0','--qy','0','--qz','0','--qw','1',
        '--frame-id','odom','--child-frame-id','base_footprint'
        ],
        output='screen'
    )

    # ★ robot_description 토픽 퍼블리셔
    desc_pub = Node(
        package='tutorial_mobile_robot',
        executable='robot_description_topic_publisher',
        parameters=[{
            'robot_description': robot_description
        }],
        output='screen'
    )

    # MuJoCo 시뮬 (유지)
    sim = Node(
        package='tutorial_mobile_robot',
        executable='mujoco_sim_node',
        parameters=[{
            'mjcf_path': mjcf,
            'loop_hz': loop_hz,
            'publish_tf': publish_tf,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # RViz2 (이제 Topic을 구독)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg, publish_tf_arg, loop_hz_arg,
        jsp, rsp, static_tf, desc_pub, sim, rviz
    ])
