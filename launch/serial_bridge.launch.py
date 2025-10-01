# launch/serial_bridge.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── 런치 인자 선언 ─────────────────────────────────────────────
    device_arg   = DeclareLaunchArgument('device',   default_value='/dev/ttyUSB0')
    baudrate_arg = DeclareLaunchArgument('baudrate', default_value='19200')

    # 프레임 헤더
    rmid_arg     = DeclareLaunchArgument('rmid',     default_value='183')  # MID_MDT
    pcid_arg     = DeclareLaunchArgument('pcid',     default_value='172')  # MID_PC
    fixed_id_arg = DeclareLaunchArgument('fixed_id', default_value='1')

    # 단위/스케일/극성
    rpm_scale_arg       = DeclareLaunchArgument('rpm_scale',           default_value='1')
    min_abs_rpm_arg     = DeclareLaunchArgument('min_abs_rpm_command', default_value='0.0')
    invert_left_arg     = DeclareLaunchArgument('invert_left',         default_value='true')
    invert_right_arg    = DeclareLaunchArgument('invert_right',        default_value='false')

    # 피드백 모드 (브로드캐스트 / 폴링)
    bc_on_start_arg     = DeclareLaunchArgument('main_data_broadcast_on_start', default_value='true')
    use_polling_arg     = DeclareLaunchArgument('use_polling_feedback', default_value='true')
    req_monitor_id_arg  = DeclareLaunchArgument('req_monitor_id', default_value='2')
    request_hz_arg      = DeclareLaunchArgument('request_hz', default_value='20.0')

    # 오돔/TF
    base_frame_arg      = DeclareLaunchArgument('base_frame_id', default_value='base_link')
    odom_frame_arg      = DeclareLaunchArgument('odom_frame_id', default_value='odom')
    publish_tf_arg      = DeclareLaunchArgument('publish_tf', default_value='true')
    loop_hz_arg         = DeclareLaunchArgument('loop_hz', default_value='100.0')
    wheel_radius_arg    = DeclareLaunchArgument('wheel_radius', default_value='0.065')
    wheel_length_arg    = DeclareLaunchArgument('wheel_length', default_value='0.4465')

    # 조인트 이름
    left_joint_arg      = DeclareLaunchArgument('left_joint_name',  default_value='left_wheel_joint')
    right_joint_arg     = DeclareLaunchArgument('right_joint_name', default_value='right_wheel_joint')

    # ── 런치 변수 ─────────────────────────────────────────────
    device        = LaunchConfiguration('device')
    baudrate      = LaunchConfiguration('baudrate')
    rmid          = LaunchConfiguration('rmid')
    pcid          = LaunchConfiguration('pcid')
    fixed_id      = LaunchConfiguration('fixed_id')
    rpm_scale     = LaunchConfiguration('rpm_scale')
    min_abs_rpm   = LaunchConfiguration('min_abs_rpm_command')
    invert_left   = LaunchConfiguration('invert_left')
    invert_right  = LaunchConfiguration('invert_right')
    bc_on_start   = LaunchConfiguration('main_data_broadcast_on_start')
    use_polling   = LaunchConfiguration('use_polling_feedback')
    req_monitor_id= LaunchConfiguration('req_monitor_id')
    request_hz    = LaunchConfiguration('request_hz')
    base_frame    = LaunchConfiguration('base_frame_id')
    odom_frame    = LaunchConfiguration('odom_frame_id')
    publish_tf    = LaunchConfiguration('publish_tf')
    loop_hz       = LaunchConfiguration('loop_hz')
    wheel_radius  = LaunchConfiguration('wheel_radius')
    wheel_length  = LaunchConfiguration('wheel_length')
    left_joint    = LaunchConfiguration('left_joint_name')
    right_joint   = LaunchConfiguration('right_joint_name')

    # ── 시리얼 브리지 노드 ─────────────────────────────────────
    serial_bridge = Node(
        package='tutorial_mobile_robot',
        executable='serial_bridge_node',
        name='serial_bridge_node',
        output='screen',
        parameters=[{
            # 포트/프레임 헤더
            'device': device,
            'baudrate': baudrate,
            'rmid': rmid,
            'pcid': pcid,
            'fixed_id': fixed_id,

            # 변환/극성
            'rpm_scale': rpm_scale,
            'min_abs_rpm_command': min_abs_rpm,
            'invert_left': invert_left,
            'invert_right': invert_right,

            # 피드백
            'main_data_broadcast_on_start': bc_on_start,
            'use_polling_feedback': use_polling,
            'req_monitor_id': req_monitor_id,
            'request_hz': request_hz,

            # 오돔/TF
            'base_frame_id': base_frame,
            'odom_frame_id': odom_frame,
            'publish_tf': publish_tf,
            'loop_hz': loop_hz,
            'wheel_radius': wheel_radius,
            'wheel_length': wheel_length,

            # 조인트 이름
            'left_joint_name': left_joint,
            'right_joint_name': right_joint
        }]
    )

    return LaunchDescription([
        # Declare arguments
        device_arg, baudrate_arg,
        rmid_arg, pcid_arg, fixed_id_arg,
        rpm_scale_arg, min_abs_rpm_arg,
        invert_left_arg, invert_right_arg,
        bc_on_start_arg, use_polling_arg,
        req_monitor_id_arg, request_hz_arg,
        base_frame_arg, odom_frame_arg,
        publish_tf_arg, loop_hz_arg,
        wheel_radius_arg, wheel_length_arg,
        left_joint_arg, right_joint_arg,

        # Node
        serial_bridge
    ])
