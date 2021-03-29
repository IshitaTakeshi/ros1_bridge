# RosbagConverter

## My Bash Aliases
```bash
alias soii="source install/setup.bash"
alias soro1="source /opt/ros/noetic/setup.bash"
alias soro2="source /opt/ros/foxy/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export ROS_LOCALHOST_ONLY=1"
```

## Building
```bash
soro2
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=1  --symlink-install --packages-skip ros1_bridge
soro1
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=1 --symlink-install --packages-select ros1_bridge --cmake-force-configure
```

## Running
```bash
soro1
soro2
soii
ros2 run ros1_bridge rosbag_converter_node --ros-args --params-file /home/mfc/projects/rosbag_converter_ws/src/ros1_bridge/params/test_param.yaml
```