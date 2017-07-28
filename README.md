# sawForceDimensionSDK
SAW wrapper for Force Dimension haptic devices - the ForceDimension SDK happens to support the devices manufactured by ForceDimension (Sigma, Omega, ...) as well as the Novint Falcon.

The Force Dimension SDK can be downloaded from the manufacturer's web site: http://www.forcedimension.com/download/sdk

# Linux permissions

The following instructions allow to avoid using `sudo` to access the USB device.  We use a `udev` rule to set the permission and optionally an owner/group.  The following example simply allows anyone to read/write using the permissions 666.

Using `dmesg`, we can find the vendor Id right afer the device is turned on:
```sh
[478324.211029] usb 2-1.5: Product: omega.x haptic device
[478324.211033] usb 2-1.5: Manufacturer: FD 3.0
[478385.766668] usb 2-1.5: USB disconnect, device number 23
[478392.620604] usb 2-1.5: new high-speed USB device number 24 using ehci-pci
[478393.276357] usb 2-1.5: New USB device found, idVendor=1451, idProduct=0402
[478393.276360] usb 2-1.5: New USB device strings: Mfr=1, Product=2, SerialNumber=0
```

For a Novint Falcon, the vendor Id might be different.  For example:
```sh
[2674365.004455] usb 1-1.4: New USB device found, idVendor=0403, idProduct=cb48
[2674365.004464] usb 1-1.4: Product: FALCON HAPTIC
```

Become superuser
```sh
sudo su -
```
then create rule:
```sh
# go to udev rules directory
cd /etc/udev/rules.d/
# create files with new rules
echo "SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"1451\", MODE=\"0666\"" > 80-usb-force-dimension.rules
echo "SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"0403\", MODE=\"0666\"" > 80-usb-novint.rules
# restart udev
udevadm control --reload-rules
```

Once this is done, test the provided examples in the SDK `bin` folder.  You should be able to run them without `sudo`. 

# ROS/Catkin build tools

This is by far the simplest solution to compile and run the examples on Linux.
See how to build cisst with ROS/Catkin tools on the cisst wiki:
https://github.com/jhu-cisst/cisst/wiki/Compiling-cisst-and-SAW-with-CMake (Make sure you go to the ROS build instructions).

When compiling the SAW Force Dimension code, you will need to specify where to find the Force Dimension SDK.  Do a first `catkin build`, this build will skip the sawForceDimensionSDK because the directory containing the SDK is not defined.   To define it, use `ccmake` in a shell/terminal that has all the ROS environment variables defined (DO NOT USE `cmake-gui`, for some reasons, it ignores the environment variables) on the build directory for the SAW Force Dimension component.  For example:
```sh
adeguet1@lcsr-qla:~/catkin_ws$ ccmake build_release/saw_force_dimension_sdk
```
In the command above, the ROS workspace is `~/catkin_ws` and the build tree is `build_release`.  You might have `devel` or `devel_debug` depending on your workspace configuration.

Once in CMake, locate `force_dimension_sdk_DIR` and make it point to the directory containing your SDK.  For example, `~/ForceDimension/sdk-3.6.0`.  Hit configure once and the two variables `force_dimension_sdk_LIBRARY_DHD` and `force_dimension_sdk_LIBRARY_DRD` should have been found automatically.

Don't forget to hit "Generate" before quitting CMake.  You should now be able to build using `catkin build --force-cmake`.   The option `--force-cmake` is required to force CMake to run for all packages that depends on the `sawForceDimensionSDK` package.

Once the packages are all built, you must first refresh your ROS environment using `source ~/catkin_ws/devel_release/setup.bash`.

```sh
sawForceDimensionSDKQtExample
```

If you also want ROS topics corresponding to the tracked tools, try:
```sh
rosrun force_dimension_ros force_dimension
```

# Windows

todo
