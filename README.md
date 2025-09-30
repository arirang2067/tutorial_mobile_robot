# tutorial_mobile_robot
크래쉬랩 모바일 로봇 제어용 튜토리얼 패키지 ver.2025

colcon build --packages-select tutorial_mobile_robot && source install/setup.bash
ros2 launch tutorial_mobile_robot mobile_robot_sim.launch.py
ros2 launch tutorial_mobile_robot tutorial_motor.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
