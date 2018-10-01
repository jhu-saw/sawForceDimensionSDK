/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2016-11-10

  (C) Copyright 2016-2018 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnUnits.h>
#include <cisstCommon/cmnCommandLineOptions.h>
#include <cisstCommon/cmnQt.h>
#include <cisstMultiTask/mtsTaskManager.h>
#include <sawForceDimensionSDK/mtsForceDimension.h>
#include <sawForceDimensionSDK/mtsForceDimensionQtWidget.h>

#include <ros/ros.h>
#include <cisst_ros_bridge/mtsROSBridge.h>

#include <QApplication>
#include <QMainWindow>


int main(int argc, char * argv[])
{
    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskClassMatching("mtsForceDimensionSDK", CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    // parse options
    cmnCommandLineOptions options;
    std::string jsonConfigFile = "";
    double rosPeriod = 2.0 * cmn_ms;

    options.AddOptionOneValue("j", "json-config",
                              "json configuration file",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &jsonConfigFile);
    options.AddOptionOneValue("p", "ros-period",
                              "period in seconds to read all tool positions (default 0.002, 2 ms, 500Hz).  There is no point to have a period higher than the device",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &rosPeriod);

    // check that all required options have been provided
    std::string errorMessage;
    if (!options.Parse(argc, argv, errorMessage)) {
        std::cerr << "Error: " << errorMessage << std::endl;
        options.PrintUsage(std::cerr);
        return -1;
    }
    std::string arguments;
    options.PrintParsedArguments(arguments);
    std::cout << "Options provided:" << std::endl << arguments << std::endl;

    // create the components
    mtsForceDimension * forceDimension = new mtsForceDimension("ForceDimensionSDK");
    forceDimension->Configure(jsonConfigFile);

    // add the components to the component manager
    mtsManagerLocal * componentManager = mtsComponentManager::GetInstance();
    componentManager->AddComponent(forceDimension);

    // ROS bridge for publishers
    mtsROSBridge * pub_bridge = new mtsROSBridge("force_dimension_pub", rosPeriod, true);
    // separate thread to spin, i.e. subscribe
    mtsROSBridge * spin_bridge = new mtsROSBridge("force_dimension_spin", 0.1 * cmn_ms, true, false);
    componentManager->AddComponent(pub_bridge);
    componentManager->AddComponent(spin_bridge);

    // create a Qt user interface
    QApplication application(argc, argv);
    cmnQt::QApplicationExitsOnCtrlC();

    // organize all widgets in a tab widget
    QTabWidget * tabWidget = new QTabWidget;

    // device
    mtsForceDimensionQtWidget * deviceWidget;

    // namespace
    std::string rosNamespace = "/force_dimension/";

    // configure all components
    typedef std::list<std::string> NamesType;
    NamesType devices;
    forceDimension->GetDeviceNames(devices);
    const NamesType::const_iterator endDevices = devices.end();
    NamesType::const_iterator device;
    for (device = devices.begin();
         device != endDevices;
         ++device) {
        // Qt
        std::string name = *device;
        deviceWidget = new mtsForceDimensionQtWidget(name + "-gui");
        deviceWidget->Configure();
        componentManager->AddComponent(deviceWidget);
        componentManager->Connect(deviceWidget->GetName(), "Device",
                                  forceDimension->GetName(), name);
        tabWidget->addTab(deviceWidget, name.c_str());

        std::string deviceNamespace = rosNamespace + name + '/';
        // motion commands
        pub_bridge->AddPublisherFromCommandRead<prmPositionCartesianGet, geometry_msgs::TransformStamped>
            (name, "measured_cp",
             deviceNamespace + "measured_cp");
        pub_bridge->AddPublisherFromCommandRead<prmVelocityCartesianGet, geometry_msgs::TwistStamped>
            (name, "measured_cv",
             deviceNamespace + "measured_cv");
        pub_bridge->AddPublisherFromCommandRead<prmForceCartesianGet, geometry_msgs::WrenchStamped>
            (name, "measured_cf",
             deviceNamespace + "measured_cf");
        pub_bridge->AddPublisherFromCommandRead<prmStateJoint, sensor_msgs::JointState>
            (name, "gripper_measured_js",
             deviceNamespace + "gripper/measured_js");
        pub_bridge->AddPublisherFromCommandRead<prmPositionCartesianGet, geometry_msgs::TransformStamped>
            (name, "setpoint_cp",
             deviceNamespace + "setpoint_cp");
        spin_bridge->AddSubscriberToCommandWrite<prmPositionCartesianSet, geometry_msgs::TransformStamped>
            (name, "servo_cp",
             deviceNamespace + "servo_cp");
        spin_bridge->AddSubscriberToCommandWrite<prmForceCartesianSet, geometry_msgs::WrenchStamped>
            (name, "servo_cf",
             deviceNamespace + "servo_cf");

        // device state
        spin_bridge->AddSubscriberToCommandWrite<std::string, std_msgs::String>
            (name, "set_device_state",
             deviceNamespace + "set_device_state");
        spin_bridge->AddPublisherFromEventWrite<std::string, std_msgs::String>
            (name, "device_state",
             deviceNamespace + "device_state");
        spin_bridge->AddServiceFromCommandRead<std::string, std_srvs::Trigger>
            (name, "device_state",
             deviceNamespace + "device_state");

        // messages
        spin_bridge->AddLogFromEventWrite(name, "Error",
                                          mtsROSEventWriteLog::ROS_LOG_ERROR);
        spin_bridge->AddLogFromEventWrite(name, "Warning",
                                          mtsROSEventWriteLog::ROS_LOG_WARN);
        spin_bridge->AddLogFromEventWrite(name, "Status",
                                          mtsROSEventWriteLog::ROS_LOG_INFO);

        // Connect
        componentManager->Connect(pub_bridge->GetName(), name,
                                  forceDimension->GetName(), name);
        componentManager->Connect(spin_bridge->GetName(), name,
                                  forceDimension->GetName(), name);

        // Buttons
        NamesType buttons;
        forceDimension->GetButtonNames(name, buttons);
        const NamesType::iterator endButtons = buttons.end();
        NamesType::iterator button;
        for (button = buttons.begin();
             button != endButtons;
             ++button) {
            // sawForceDimension button names are device-button, use device/button for ROS
            std::string rosButton = *button;
            std::replace(rosButton.begin(), rosButton.end(), '-', '/');
            spin_bridge->AddPublisherFromEventWrite<prmEventButton, sensor_msgs::Joy>
                (*button, "Button", "/force_dimension/" + rosButton);
            componentManager->Connect(spin_bridge->GetName(), *button,
                                      forceDimension->GetName(), *button);
        }
    }

    // create and start all components
    componentManager->CreateAllAndWait(5.0 * cmn_s);
    componentManager->StartAllAndWait(5.0 * cmn_s);

    // run Qt user interface
    tabWidget->show();
    application.exec();

    // kill all components and perform cleanup
    componentManager->KillAllAndWait(5.0 * cmn_s);
    componentManager->Cleanup();

    cmnLogger::Kill();

    return 0;
}
