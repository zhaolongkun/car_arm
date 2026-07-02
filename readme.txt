先进入ros2_ws里面：
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch version_car_sim sim.launch.py gui:=true rviz:=true world_file:=car_01.world rviz_config_file:=car_01.rviz auto_start:=false



在第二个终端里面执行

#设置起点：
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

ros2 topic pub --once /start_pose_2d geometry_msgs/msg/Pose2D "{x: -10.0, y: -10.0, theta: 0.0}"


#在第三个终端里面执行

#设置终点：
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 10.0, y: 10.0, z: 0.0}, orientation: {w: 1.0}}}"


#启动小车

ros2 topic pub --once /start_navigation std_msgs/msg/Bool "{data: true}"












第一个终端：
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch version_car_sim d435i_vision_sim.launch.py gui:=true rviz:=true auto_start:=false

第二个终端：
cd ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

#设置起点：
ros2 topic pub --once /start_pose_2d geometry_msgs/msg/Pose2D "{x: -10, y: -10, theta: 0.0}"

#设置终点：
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 10, y: 10, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}"

#开始运行：
ros2 topic pub --once /start_navigation std_msgs/msg/Bool "{data: true}"