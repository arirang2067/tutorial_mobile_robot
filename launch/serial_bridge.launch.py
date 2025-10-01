# launch/serial_bridge.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── 런치 인자 선언 ─────────────────────────────────────────────────────────
    device_arg   = DeclareLaunchArgument('device',   default_value='/dev/ttyUSB0')
    baudrate_arg = DeclareLaunchArgument('baudrate', default_value='19200')

    # 프레임 헤더(옛 코드 레이아웃과 동일): [RMID][PCID][ID(=fixed_id)]
    rmid_arg     = DeclareLaunchArgument('rmid',     default_value='183')  # MID_MDT
    pcid_arg     = DeclareLaunchArgument('pcid',     default_value='172')  # MID_PC
    fixed_id_arg = DeclareLaunchArgument('fixed_id', default_value='1')    # 항상 1 또는 0xFE

    # 단위/스케일/극성
    rpm_scale_arg           = DeclareLaunchArgument('rpm_scale',           default_value='1')     # 보드가 rpm×10이면 10
    min_abs_rpm_cmd_arg     = DeclareLaunchArgument('min_abs_rpm_command', default_value='0.0')   # 너무 작은 값 스냅(예: 5.0)
    invert_left_arg         = DeclareLaunchArgument('invert_left',         default_value='true')
    invert_right_arg        = DeclareLaunchArgument('invert_right',        default_value='false')

    # PID 210 방송 요청 옵션
    bc_on_start_arg         = DeclareLaunchArgument('main_data_broadcast_on_start', default_value='true')
    req_monitor_id_arg      = DeclareLaunchArgument('req_monitor_id',              default_value='2')  # REQUEST_PNT_MAIN_DATA

    # ── 런치 변수 ─────────────────────────────────────────────────────────────
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
    req_monitor_id= LaunchConfiguration('req_monitor_id')

    # ── 시리얼 브리지 노드 ───────────────────────────────────────────────────
    serial_bridge = Node(
        package='tutorial_mobile_robot',
        executable='serial_bridge_node',
        name='serial_bridge_node',
        output='screen',
        parameters=[{
            # 포트
            'device': device,
            'baudrate': baudrate,

            # 프레임 헤더
            'rmid': rmid,
            'pcid': pcid,
            'fixed_id': fixed_id,

            # 변환/극성
            'rpm_scale': rpm_scale,
            'min_abs_rpm_command': min_abs_rpm,
            'invert_left': invert_left,
            'invert_right': invert_right,

            # 방송(피드백)
            'main_data_broadcast_on_start': bc_on_start,
            'req_monitor_id': req_monitor_id,

            # 내부 디버그(필요 시 노드 파라미터로 추가 가능)
            'log_hex': True
        }]
    )

    return LaunchDescription([
        device_arg, baudrate_arg,
        rmid_arg, pcid_arg, fixed_id_arg,
        rpm_scale_arg, min_abs_rpm_cmd_arg,
        invert_left_arg, invert_right_arg,
        bc_on_start_arg, req_monitor_id_arg,
        serial_bridge
    ])
