/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2016-11-10

  (C) Copyright 2016-2017 Johns Hopkins University (JHU), All Rights Reserved.

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
    double rosPeriod = 10.0 * cmn_ms;

    options.AddOptionOneValue("j", "json-config",
                              "json configuration file",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &jsonConfigFile);
    options.AddOptionOneValue("p", "ros-period",
                              "period in seconds to read all tool positions (default 0.01, 10 ms, 100Hz).  There is no point to have a period higher than the tracker component",
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

    // ROS bridge
    mtsROSBridge * rosBridge = new mtsROSBridge("ForceDimensionBridge", rosPeriod, true, false);
    componentManager->AddComponent(rosBridge);

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
        deviceWidget = new mtsForceDimensionQtWidget(*device + "-gui");
        deviceWidget->Configure();
        componentManager->AddComponent(deviceWidget);
        componentManager->Connect(deviceWidget->GetName(), "Device",
                                  forceDimension->GetName(), *device);
        tabWidget->addTab(deviceWidget, (*device).c_str());

        rosBridge->AddPublisherFromCommandRead<prmPositionCartesianGet, geometry_msgs::PoseStamped>
            (*device, "GetPositionCartesian",
             rosNamespace + *device + "/position_cartesian_current");
        // ROS
        rosBridge->AddPublisherFromCommandRead<prmVelocityCartesianGet, geometry_msgs::TwistStamped>
            (*device, "GetVelocityCartesian",
             rosNamespace + *device + "/twist_body_current");
        rosBridge->AddPublisherFromCommandRead<prmForceCartesianGet, geometry_msgs::WrenchStamped>
            (*device, "GetWrenchBody",
             rosNamespace + *device + "/wrench_body_current");
        rosBridge->AddPublisherFromCommandRead<prmStateJoint, sensor_msgs::JointState>
            (*device, "GetStateGripper",
             rosNamespace + *device + "/state_gripper_current");
        // Connect
        componentManager->Connect(rosBridge->GetName(), *device,
                                  forceDimension->GetName(), *device);

        // Buttons
        NamesType buttons;
        forceDimension->GetButtonNames(*device, buttons);
        const NamesType::iterator endButtons = buttons.end();
        NamesType::iterator button;
        for (button = buttons.begin();
             button != endButtons;
             ++button) {
            // sawForceDimension button names are device-button, use device/button for ROS
            std::string rosButton = *button;
            std::replace(rosButton.begin(), rosButton.end(), '-', '/');
            rosBridge->AddPublisherFromEventWrite<prmEventButton, sensor_msgs::Joy>
                (*button, "Button", "/force_dimension/" + rosButton);
            componentManager->Connect(rosBridge->GetName(), *button,
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
