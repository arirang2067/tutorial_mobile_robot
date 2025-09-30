# launch/serial_bridge.launch.py
#
# 목적: 실제 하드웨어에서 모터 제어용 시리얼 브리지 노드 실행
#  - /wheel_speeds_cmd 토픽을 받아 시리얼 패킷 전송 (SerialCom 내부 사용)
#
# 사용 예:
#   ros2 launch tutorial_mobile_robot serial_bridge.launch.py
#   ros2 launch tutorial_mobile_robot serial_bridge.launch.py device:=/dev/ttyUSB1 baudrate:=921600
#
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ── 런치 인자 (필요시 수정)
    device_arg        = DeclareLaunchArgument('device',        default_value='/dev/ttyUSB0')
    baudrate_arg      = DeclareLaunchArgument('baudrate',      default_value='115200')
    pc_id_arg         = DeclareLaunchArgument('pc_id',         default_value='85')   # 0x55
    mdt_id_arg        = DeclareLaunchArgument('mdt_id',        default_value='170')  # 0xAA
    mdui_id_arg       = DeclareLaunchArgument('mdui_id',       default_value='171')  # 0xAB
    left_id_arg       = DeclareLaunchArgument('left_motor_id',  default_value='1')
    right_id_arg      = DeclareLaunchArgument('right_motor_id', default_value='2')
    pid_speed_arg     = DeclareLaunchArgument('pid_set_speed', default_value='16')   # 0x10
    tx_rate_arg       = DeclareLaunchArgument('tx_rate_hz',    default_value='100.0')
    timeout_arg       = DeclareLaunchArgument('timeout_no_cmd_sec', default_value='0.2')

    # ── 런치 변수
    device        = LaunchConfiguration('device')
    baudrate      = LaunchConfiguration('baudrate')
    pc_id         = LaunchConfiguration('pc_id')
    mdt_id        = LaunchConfiguration('mdt_id')
    mdui_id       = LaunchConfiguration('mdui_id')
    left_motor_id = LaunchConfiguration('left_motor_id')
    right_motor_id= LaunchConfiguration('right_motor_id')
    pid_set_speed = LaunchConfiguration('pid_set_speed')
    tx_rate_hz    = LaunchConfiguration('tx_rate_hz')
    timeout_sec   = LaunchConfiguration('timeout_no_cmd_sec')

    # ── 시리얼 브리지 노드
    serial_bridge = Node(
        package='tutorial_mobile_robot',
        executable='serial_bridge_node',
        name='serial_bridge_node',
        parameters=[{
            'device': device,
            'baudrate': baudrate,
            'pc_id': pc_id,
            'mdt_id': mdt_id,
            'mdui_id': mdui_id,
            'left_motor_id': left_motor_id,
            'right_motor_id': right_motor_id,
            'pid_set_speed': pid_set_speed,
            'tx_rate_hz': tx_rate_hz,
            'timeout_no_cmd_sec': timeout_sec
        }],
        output='screen'
    )

    return LaunchDescription([
        device_arg, baudrate_arg,
        pc_id_arg, mdt_id_arg, mdui_id_arg,
        left_id_arg, right_id_arg, pid_speed_arg,
        tx_rate_arg, timeout_arg,
        serial_bridge
    ])
