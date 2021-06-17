# sawForceDimensionSDK
SAW wrapper for Force Dimension haptic devices - the ForceDimension SDK happens to support the devices manufactured by ForceDimension (Sigma, Omega, ...) as well as the Novint Falcon.  This code compile on Linux and Windows.  This repositor provides a core component as well as:
* Example application with Qt based GUI
* ROS node (also with Qt based GUI)
* ROS based python client

# Links
 * License: http://github.com/jhu-cisst/cisst/blob/master/license.txt
 * JHU-LCSR software: http://jhu-lcsr.github.io/software/

# Dependencies
 * cisst libraries: https://github.com/jhu-cisst/cisst
 * Force Dimension SDK. Downloaded from the manufacturer's web site: http://www.forcedimension.com/download/sdk
 * Qt for user interface
 * ROS (optional)
 * ROS CRTK (optional) 

# Compilation and configuration

## Linux permissions

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

## ROS/Catkin build tools

This is by far the simplest solution to compile and run the examples on Linux.
See how to build cisst with ROS/Catkin tools on the cisst wiki:
https://github.com/jhu-cisst/cisst/wiki/Compiling-cisst-and-SAW-with-CMake (Make sure you go to the ROS build instructions).

To retrieve the code and make sure you have all the repositories you need, you can use the `rosinstall` file in the `ros` directory.  To do so, go in your catkin workspace `src` directory and run:
```sh
wstool init
wstool merge https://raw.githubusercontent.com/jhu-saw/sawForceDimensionSDK/master/ros/force_dimension.rosinstall
wstool update
```

When compiling the SAW Force Dimension code, you will need to specify where to find the Force Dimension SDK.  Do a first `catkin build`, this build will skip *sawForceDimensionSDK* because the directory containing the SDK is not known.   To define it, use `ccmake` in a shell/terminal that has all the ROS environment variables defined (DO NOT USE `cmake-gui`, for some reasons, it ignores the environment variables) on the build directory for the SAW Force Dimension component.  For example:
```sh
adeguet1@lcsr-qla:~/catkin_ws$ ccmake build/saw_force_dimension_sdk
```
In the command above, the ROS workspace is `~/catkin_ws` and the build tree is `build`.

Once in CMake, locate `force_dimension_sdk_DIR` and make it point to the directory containing your SDK.  For example, `~/ForceDimension/sdk-3.6.0`.  Hit configure once and the two variables `force_dimension_sdk_LIBRARY_DHD` and `force_dimension_sdk_LIBRARY_DRD` should have been found automatically.

Don't forget to hit "Generate" before quitting CMake.  You should now be able to build using `catkin build --force-cmake`.   The option `--force-cmake` is required to force CMake to run for all packages that depends on the `sawForceDimensionSDK` package.

Once the packages are all built, you must first refresh your ROS environment using `source ~/catkin_ws/devel/setup.bash`.

# Examples

## Main example

The main example provided is `sawForceDimensionSDKQtExample`.  The command line options are:
```sh
sawForceDimensionSDKQtExample:
 -j <value>, --json-config <value> : json configuration file (optional)
 -m, --component-manager : JSON files to configure component manager (optional)
 -D, --dark-mode : replaces the default Qt palette with darker colors (optional)
```

To run the example with a configuration file, use:
```sh
sawForceDimensionSDKQtExample -j myconfig.json
```

The configuration file can be used to rename the devices based on their serial number.  Note that for the Novint Falcon the SDK in unable to find the serial number so you have to use the serial number `0`:
```json
{
    "devices":
    [
        {
            "name": "left",
            "serial": 0
        }
        ,
        {
            "name": "right",
            "serial": 0
        }
    ]
}
```

## ROS

If you also want ROS topics corresponding to the tracked tools, try:
```sh
rosrun force_dimension_ros force_dimension
```

## ROS CRTK Python and Matlab client

Once you have the ATI Force Sensor ROS node working, you can create your own ROS subscriber in different languages, including C++, Python, Matlab...  If you want to use Python or Matlab, the CRTK client libraries might be useful:
* [CRTK Python](https://github.com/collaborative-robotics/crtk_python_client)
* [CRTK Matlab](https://github.com/collaborative-robotics/crtk_matlab_client)

A simple ROS client is provided in this package.  Once the ROS `force_dimension` node is started, one can do (in Python):
```python
import force_dimension
f = force_dimension.arm('Falcon00') # use rostopic list to verify device's name
# get the current cartesian position
p = f.measured_cp()
p
[[           1,           0,           0;
            0,           1,           0;
            0,           0,           1]
[   -0.040937, -0.00489694,   0.0107565]]
# apply a 5 newton force in x direction
f.body.servo_cf([5.0, 0.0, 0.0, 0.0, 0.0, 0.0])
# reset the force to zero
f.body.servo_cf([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

```

## Other "middleware"

Besides ROS, the ATI Force Sensor component can also stream data to your application using the *sawOpenIGTLink* or *sawSocketStreamer* components.  See:
* [sawOpenIGTLink](https://github.com/jhu-saw/sawOpenIGTLink)
* [sawSocketStreamer](https://github.com/jhu-saw/sawSocketStreamer)
