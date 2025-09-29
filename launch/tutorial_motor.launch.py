# launch/tutorial_motor.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="tutorial_ros2_motor",
            executable="tutorial_motor_node",
            name="tutorial_ros2_motor_node",
            output="screen",
            parameters=[{
                # 프레임/루프
                "loop_hz": 100.0,
                "publish_tf": True,
                "base_frame_id": "base_link",
                "odom_frame_id": "odom",
                "left_joint_name": "wheel_left_joint",
                "right_joint_name": "wheel_right_joint",
                # Kinematics
                "wheel_radius": 0.08,
                "wheel_length": 0.42,
                "gear_ratio": 4.0,
                "max_rpm": 350,
                # Controller (step 기본)
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
    ])
