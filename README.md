# RosbagConverter
This repository has been p̶i̶l̶l̶a̶g̶e̶d̶ modified to be able to convert ROS1 bag files to ROS2 bag files.

The `rosbag_converter_node` executable in `ros1_bridge` package converts the ROS1 bag files to `.proto_rosbag2`
format. Then you should use https://github.com/leo-drive/proto_to_rosbag2 package and its instructions to convert
that into ROS2 bag files.

## Why not just convert it into ROS2 bag in the same executable?
Linker error nightmares. I simply couldn't link an executable containing both `rosbag`, `rosbag2_cpp` 
and `ros1_bridge` libraries.
If you find a way, please show me the way :)

## Why not just convert it into ROS2 bag in the same workspace?
This repository works just fine with `noetic` and `foxy` and this repository requires weird sourcing
mechanisms and orders to work at all.

On the other hand, the other workspace requires `Rolling` packages
for `rosbag2_cpp` to function writing stuff.

## My Bash Aliases
```bash
# You can add following to your ~/.bash_aliases file (if it doesn't exist, create one) to make your life easier
alias soii="source install/setup.bash"
alias soro1="source /opt/ros/noetic/setup.bash"
alias soro2="source /opt/ros/foxy/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export ROS_LOCALHOST_ONLY=1"
```

## Building
In order for this workspace to work, you should be very careful with your terminals, shells, `.bashrc` file.

You need a terminal which hasn't sourced any ROS1 or ROS2 or any packages at all. This is very important.

To confirm do `echo ${ROS_DISTRO}` and it should return blank line. If it says `noetic` or `foxy`
check who sourced it and remove/comment it.

```bash
mkdir -p ~/rosbag_converter_ws/src
cd ~/rosbag_converter_ws/src
git clone https://github.com/leo-drive/ros1_bridge.git
cd ~/rosbag_converter_ws

# The order is important
soro2
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=1  --symlink-install --packages-skip ros1_bridge
soro1
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=1 --symlink-install --packages-select ros1_bridge --cmake-force-configure
```
Once you've built it, if you make small changes to the code, to recompile, repeating last step which is
`colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=1 --symlink-install --packages-select ros1_bridge --cmake-force-configure` 
is okay, it will compile just fine. (Make sure current terminal has ROS2 and ROS1 sourced in order above.)

## Running
```bash
Edit this file with your directories:
~/rosbag_converter_ws/src/ros1_bridge/params/test_param.yaml
# defaults:
    path_in_ros1_bag_file: "/home/mfc/bags/038/038.bag" # This file will be used read only
    path_out_ros2_serialized_binary: "/home/mfc/bags/038/038.proto_rosbag2" # This file will be overwritten if it exists
    print_type_correspondences_1_to_2: false # Use this to print what ROS1<->ROS2 conversions are available for you
```
```bash
soro1
soii
soro1 # this is required for libclass_loader.so and a couple more things to link correctly
ros2 run ros1_bridge rosbag_converter_node --ros-args --params-file ~/rosbag_converter_ws/src/ros1_bridge/params/test_param.yaml
```
Now final structure should look like this:
```bash
mfc@mfc-SG13:~/bags/038$ tree
.
├── 038.bag
└── 038.proto_rosbag2
```
Now you can proceed to follow instructions at https://github.com/leo-drive/proto_to_rosbag2 to convert
`.proto_rosbag2` to ROS2 bag format(`db3`).
