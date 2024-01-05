Change log
==========

2.1.0 (2024-01-05)
==================

* API changes:
  * The `component` directory is now moved under `core` to avoid having a `CMakeLists.txt` at the top level (for ROS2/colcon)
* Deprecated features:
  * wstool support has been dropped, replaced by vcs (see files in `vcs` directory)
* New features:
  * CRTK based client and example of Python script (`xyz-motions.py`)
  * `ros` subdirectory can now be compiled for ROS1/catkin or ROS2/colcon!
* Bug fixes:
  * Fixed CMake to use latest cisst CMake macros, install targets properly defined

2.0.0 (2021-06-18)
==================

* API changes:
  * Using CRTK naming convention (i.e. forces are reported using `measured_cf`)
* Deprecated features:
  * None
* New features:
  * Better ROS bridge using cisst ROS CRTK class
  * Use Qt widgets from cisst as much as possible
  * rosinstall file to get dependencies using `wstool`
  * Support cisstMultiTask manager configuration files to use with other middleware (e.g. OpenIGTLink)
* Bug fixes:
  * None

1.1.0 (2018-05-16)
==================

* API changes:
  * None
* Deprecated features:
  * None
* New features:
  * Get velocity and effort for gripper state
  * Updated Qt widgets for cartesian position and wrench
  * Added ROS topics for buttons
  * ROS bridge should now exit on ctrl-c
* Bug fixes:
  * None

1.0.0 (2017-11-08)
==================

* API changes:
  * None
* Deprecated features:
  * None
* New features:
  * Initial release
  * cisst/SAW wrapper for Force Dimension SDK to support Noving Falcon and Force Dimension haptic devices
  * Comes with a Qt widget to display current status and buttons to lock/unlock arm
  * Simple ROS bridge to publish current state of device, subscribers to control device can be added if needed, do not hesitate to contact the developer if you need ROS subscribers
* Bug fixes:
  * None
