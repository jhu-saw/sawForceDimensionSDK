# sawForceDimensionSDK
SAW wrapper for Force Dimension haptic devices

# ROS/Catkin build tools

This is by far the simplest solution to compile and run the examples on Linux.
See how to build cisst with ROS/Catkin tools on the cisst wiki:
https://github.com/jhu-cisst/cisst/wiki/Compiling-cisst-and-SAW-with-CMake

When compiling the SAW Force Dimension code, you will need to specify where to find the Force Dimension SDK.  Do a first `catkin build`, this build will fail because the directory containing the SDK is not defined.   To define it, use `ccmake` or `cmake-gui` on the build directory for the SAW Force Dimension component.  For example:
```sh
adeguet1@lcsr-qla:~/catkin_ws$ cmake-gui build_release/saw_force_dimension_sdk
```
In the command above, the ROS workspace is `~/catkin_ws` and the build tree is `build_release`.  You might have `devel` or `devel_debug` depending on your workspace configuration.

Once in CMake, locate `force_dimension_sdk_DIR` and make it point to the directory containing your SDK.  TO BE UPDATED:   For example, `~/fusionTrack_v2.3_gcc-4.7`.  Hit configure once and the two variables `atracsys_LIBRARY_device` and `atracsys_LIBRARY_fusionTrack` should have been found automatically.

Don't forget to hit "Generate" before quitting CMake.  You should now be able to build using `catkin build --force-cmake`.   The option `--force-cmake` is required to force CMake to run for all packages that depends on the `sawForceDimensionSDK` package.

Once the packages are all built, you must first refresh your ROS environment using `source ~/catkin_ws/devel_release/setup.bash`.

```sh
sawForceDimensionSDKQtExample -j config003.json
```

If you also want ROS topics corresponding to the tracked tools, try:
```sh
rosrun force_dimension_ros force_dimension
```

# Windows

todo
