#  Author(s):  Anton Deguet
#  Created on: 2021-03-04

# (C) Copyright 2021 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

import crtk
import rospy
import PyKDL

class arm(object):
    """Simple arm API wrapping around ROS messages
    """

    # initialize the arm
    def __init__(self, arm_name, ros_namespace = '', expected_interval = 0.01):
        # base class constructor in separate method so it can be called in derived classes
        self.__init_arm(arm_name, ros_namespace, expected_interval)


    def __init_arm(self, arm_name, ros_namespace, expected_interval):
        """Constructor.  This initializes a few data members.It
        requires a arm name, this will be used to find the ROS
        topics for the arm being controlled.  For example if the
        user wants `arm`, the ROS topics will be from the namespace
        `arm`"""
        # data members, event based
        self.__arm_name = arm_name
        self.__ros_namespace = ros_namespace
        self.__full_ros_namespace = ros_namespace + arm_name

        # crtk features
        self.__crtk_utils = crtk.utils(self, self.__full_ros_namespace, expected_interval)

        # add crtk features that we need and are supported by the dVRK
        self.__crtk_utils.add_operating_state()
        self.__crtk_utils.add_measured_js()
        self.__crtk_utils.add_measured_cp()
        self.__crtk_utils.add_measured_cv()
        self.__crtk_utils.add_servo_cp()
        self.__crtk_utils.add_servo_cf()

        # create node
        if not rospy.get_node_uri():
            rospy.init_node('arm_api', anonymous = True, log_level = rospy.WARN)
        else:
            rospy.logdebug(rospy.get_caller_id() + ' -> ROS already initialized')

    def name(self):
        return self.__arm_name

    def namespace(self):
        return self.__full_ros_namespace
