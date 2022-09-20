#!/usr/bin/env python

# Author: Anton Deguet
# Date: 2021-03-04

# (C) Copyright 2021 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

# Start the force_dimension node
# > rosrun force_dimension_ros force_dimension

# To communicate with the arm using ROS topics, run this script
# > rosrun force_dimension_ros xyz-motion.py -a Falcon00

import force_dimension
import math
import sys
import time
import rospy
import numpy
import PyKDL
import argparse

# example of application using arm.py
class example_application:

    # configuration
    def configure(self, robot_name, expected_interval):
        self.expected_interval = expected_interval
        self.arm = force_dimension.arm(arm_name = robot_name,
                                       expected_interval = expected_interval)

    # homing example
    def home(self):
        print('starting enable')
        if not self.arm.enable(10):
            sys.exit('failed to enable within 10 seconds')
        print('starting home')
        if not self.arm.home(10):
            sys.exit('failed to home within 10 seconds')

    # direct cartesian control example
    def xyz_motion(self):
        print('starting xyz motion')
        # create a new goal starting with current position
        initial_cartesian_position = PyKDL.Frame()
        initial_cartesian_position.p = self.arm.measured_cp().p
        initial_cartesian_position.M = self.arm.measured_cp().M
        goal = PyKDL.Frame()
        goal.p = self.arm.measured_cp().p
        goal.M = self.arm.measured_cp().M
        # motion parameters
        amplitude = 0.02 # 4 cm total
        duration = 3  # 3 seconds
        samples = duration / self.expected_interval
        print('starting x motion')
        for i in range(int(samples)):
            goal.p[0] =  initial_cartesian_position.p[0] + amplitude *  (1.0 - math.cos(i * math.radians(360.0) / samples))
            self.arm.servo_cp(goal)
            rospy.sleep(self.expected_interval)
        print('starting y motion')
        for i in range(int(samples)):
            goal.p[1] =  initial_cartesian_position.p[1] + amplitude *  (1.0 - math.cos(i * math.radians(360.0) / samples))
            self.arm.servo_cp(goal)
            rospy.sleep(self.expected_interval)
        print('starting z motion')
        for i in range(int(samples)):
            goal.p[2] =  initial_cartesian_position.p[2] + amplitude *  (1.0 - math.cos(i * math.radians(360.0) / samples))
            self.arm.servo_cp(goal)
            rospy.sleep(self.expected_interval)
        # release the arm by sending zero wrench
        wrench = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.arm.body.servo_cf(wrench)

    # main method
    def run(self):
        self.home()
        self.xyz_motion()

if __name__ == '__main__':
    # ros init node so we can use default ros arguments (e.g. __ns:= for namespace)
    rospy.init_node('force_dimension_xyz_motion', anonymous=True)
    # strip ros arguments
    argv = rospy.myargv(argv=sys.argv)

    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--arm', type=str, required=True,
                        help = 'arm name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace')
    parser.add_argument('-i', '--interval', type=float, default=0.01,
                        help = 'expected interval in seconds between messages sent by the device')
    args = parser.parse_args(argv[1:]) # skip argv[0], script name

    application = example_application()
    application.configure(args.arm, args.interval)
    application.run()
