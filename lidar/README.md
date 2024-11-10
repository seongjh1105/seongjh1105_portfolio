# seongjh1105_portfolio

1. cd ros2_ws/
2. colcon build --packages-select lidar
3. source ~/ros2_ws/install/setup.bash
4-1. ros2 launch lidar lidar_straight.launch.py
4-2. ros2 bag play ~/ros2_ws/dataset/test001/
5-1. ros2 launch lidar lidar_curved.launch.py
5-2. ros2 bag play ~/ros2_ws/dataset/test006/

