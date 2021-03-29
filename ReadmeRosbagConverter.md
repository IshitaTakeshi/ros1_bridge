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
soro1 # this shouldnt be here but for now it is necessary
ros2 run ros1_bridge rosbag_converter_node --ros-args --params-file /home/mfc/projects/rosbag_converter_ws/src/ros1_bridge/params/test_param.yaml
```

Added sourcing of ros1 to the end else it gives linker error similar to this issue: https://github.com/ros2/ros1_bridge/issues/205#issuecomment-698323248

```bash
/home/mfc/projects/rosbag_converter_ws/install/ros1_bridge/lib/ros1_bridge/rosbag_converter_node: symbol lookup error: /opt/ros/noetic/lib/librosbag_storage.so: undefined symbol: _ZN12class_loader23MultiLibraryClassLoader27getAllAvailableClassLoadersEv
```
When I further investigate it:
```bash
mfc@mfc-SG13:~/projects/rosbag_converter_ws$ ldd /opt/ros/noetic/lib/librosbag_storage.so 
	linux-vdso.so.1 (0x00007ffe68bf6000)
	libclass_loader.so => /opt/ros/foxy/lib/libclass_loader.so (0x00007fc1cdb3b000)
	librosconsole.so => /opt/ros/noetic/lib/librosconsole.so (0x00007fc1cdad8000)
	libroslib.so => /opt/ros/noetic/lib/libroslib.so (0x00007fc1cdabd000)
	libboost_filesystem.so.1.71.0 => /lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0 (0x00007fc1cda58000)
	libtinyxml2.so.6 => /lib/x86_64-linux-gnu/libtinyxml2.so.6 (0x00007fc1cda41000)
	libroslz4.so => /opt/ros/noetic/lib/libroslz4.so (0x00007fc1cda38000)
	librostime.so => /opt/ros/noetic/lib/librostime.so (0x00007fc1cda0c000)
	libcpp_common.so => /opt/ros/noetic/lib/libcpp_common.so (0x00007fc1cd9fe000)
	libconsole_bridge.so.0.4 => /lib/x86_64-linux-gnu/libconsole_bridge.so.0.4 (0x00007fc1cd9f8000)
	libbz2.so.1.0 => /lib/x86_64-linux-gnu/libbz2.so.1.0 (0x00007fc1cd9e5000)
	libstdc++.so.6 => /lib/x86_64-linux-gnu/libstdc++.so.6 (0x00007fc1cd804000)
	libgcc_s.so.1 => /lib/x86_64-linux-gnu/libgcc_s.so.1 (0x00007fc1cd7e7000)
	libc.so.6 => /lib/x86_64-linux-gnu/libc.so.6 (0x00007fc1cd5f5000)
	libconsole_bridge.so.1.0 => /opt/ros/foxy/lib/x86_64-linux-gnu/libconsole_bridge.so.1.0 (0x00007fc1cd5ef000)
	librcpputils.so => /opt/ros/foxy/lib/librcpputils.so (0x00007fc1cd5e6000)
	librosconsole_log4cxx.so => /opt/ros/noetic/lib/librosconsole_log4cxx.so (0x00007fc1cd5c6000)
	librosconsole_backend_interface.so => /opt/ros/noetic/lib/librosconsole_backend_interface.so (0x00007fc1cd5c1000)
	liblog4cxx.so.10 => /lib/x86_64-linux-gnu/liblog4cxx.so.10 (0x00007fc1cd3e1000)
	libboost_regex.so.1.71.0 => /lib/x86_64-linux-gnu/libboost_regex.so.1.71.0 (0x00007fc1cd2e1000)
	libpthread.so.0 => /lib/x86_64-linux-gnu/libpthread.so.0 (0x00007fc1cd2be000)
	librospack.so => /opt/ros/noetic/lib/librospack.so (0x00007fc1cd26f000)
	liblz4.so.1 => /lib/x86_64-linux-gnu/liblz4.so.1 (0x00007fc1cd24e000)
	libm.so.6 => /lib/x86_64-linux-gnu/libm.so.6 (0x00007fc1cd0fd000)
	/lib64/ld-linux-x86-64.so.2 (0x00007fc1cdbd9000)
	librcutils.so => /opt/ros/foxy/lib/librcutils.so (0x00007fc1cd0e5000)
	libapr-1.so.0 => /lib/x86_64-linux-gnu/libapr-1.so.0 (0x00007fc1cd0ac000)
	libaprutil-1.so.0 => /lib/x86_64-linux-gnu/libaprutil-1.so.0 (0x00007fc1cd07e000)
	libicui18n.so.66 => /opt/spinnaker/lib/libicui18n.so.66 (0x00007fc1ccd7f000)
	libicuuc.so.66 => /opt/spinnaker/lib/libicuuc.so.66 (0x00007fc1ccb97000)
	libboost_program_options.so.1.71.0 => /lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0 (0x00007fc1ccb08000)
	libpython3.8.so.1.0 => /lib/x86_64-linux-gnu/libpython3.8.so.1.0 (0x00007fc1cc5b2000)
	libdl.so.2 => /lib/x86_64-linux-gnu/libdl.so.2 (0x00007fc1cc5ac000)
	libuuid.so.1 => /lib/x86_64-linux-gnu/libuuid.so.1 (0x00007fc1cc5a3000)
	libcrypt.so.1 => /lib/x86_64-linux-gnu/libcrypt.so.1 (0x00007fc1cc566000)
	libexpat.so.1 => /lib/x86_64-linux-gnu/libexpat.so.1 (0x00007fc1cc538000)
	libicudata.so.66 => /opt/spinnaker/lib/libicudata.so.66 (0x00007fc1caa77000)
	libz.so.1 => /lib/x86_64-linux-gnu/libz.so.1 (0x00007fc1caa5b000)
	libutil.so.1 => /lib/x86_64-linux-gnu/libutil.so.1 (0x00007fc1caa56000)
mfc@mfc-SG13:~/projects/rosbag_converter_ws$ ldd /opt/ros/noetic/lib/librosbag_storage.so | grep foxy
	libclass_loader.so => /opt/ros/foxy/lib/libclass_loader.so (0x00007fa855789000)
	libconsole_bridge.so.1.0 => /opt/ros/foxy/lib/x86_64-linux-gnu/libconsole_bridge.so.1.0 (0x00007fa85523d000)
	librcpputils.so => /opt/ros/foxy/lib/librcpputils.so (0x00007fa855234000)
	librcutils.so => /opt/ros/foxy/lib/librcutils.so (0x00007fa854d33000)
```

4 .so files are linked to foxy :(

For now I re source the ros1 to fix linking but probably will relive problems when I try to use ros2 bag.