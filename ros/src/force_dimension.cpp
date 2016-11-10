/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2016-11-10

  (C) Copyright 2016 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnUnits.h>
#include <cisstCommon/cmnCommandLineOptions.h>
#include <cisstMultiTask/mtsTaskManager.h>
#include <sawForceDimensionSDK/mtsForceDimensionSDK.h>
#include <sawForceDimensionSDK/mtsForceDimensionSDKToolQtWidget.h>
#include <sawForceDimensionSDK/mtsForceDimensionSDKStrayMarkersQtWidget.h>

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
    mtsForceDimensionSDK * tracker = new mtsForceDimensionSDK("FusionTrack");
    tracker->Configure(jsonConfigFile);

    // add the components to the component manager
    mtsManagerLocal * componentManager = mtsComponentManager::GetInstance();
    componentManager->AddComponent(tracker);

    // ROS bridge
    mtsROSBridge * rosBridge = new mtsROSBridge("ForceDimensionBridge", rosPeriod, true);

    // create a Qt user interface
    QApplication application(argc, argv);

    // organize all widgets in a tab widget
    QTabWidget * tabWidget = new QTabWidget;

    // stray markers
    mtsForceDimensionSDKStrayMarkersQtWidget * strayMarkersWidget;
    strayMarkersWidget = new mtsForceDimensionSDKStrayMarkersQtWidget("StrayMarkers-GUI");
    strayMarkersWidget->Configure();
    componentManager->AddComponent(strayMarkersWidget);
    componentManager->Connect(strayMarkersWidget->GetName(), "Controller",
                              tracker->GetName(), "Controller");
    tabWidget->addTab(strayMarkersWidget, "Stray Markers");

    // tools
    std::string toolName;
    mtsForceDimensionSDKToolQtWidget * toolWidget;

    // configure all components
    for (size_t tool = 0; tool < tracker->GetNumberOfTools(); tool++) {
        toolName = tracker->GetToolName(tool);
        // ROS publisher
        std::string topicName = toolName;
        std::replace(topicName.begin(), topicName.end(), '-', '_');
        rosBridge->AddPublisherFromCommandRead<prmPositionCartesianGet, geometry_msgs::PoseStamped>
            (toolName, "GetPositionCartesian",
             "/force_dimension/" + topicName);
        // Qt Widget
        toolWidget = new mtsForceDimensionSDKToolqtwidget(toolName + "-GUI");
        toolWidget->Configure();
        componentManager->AddComponent(toolWidget);
        componentManager->Connect(toolWidget->GetName(), "Tool",
                                  tracker->GetName(), toolName);
        tabWidget->addTab(toolWidget, toolName.c_str());
    }

    // add ROS bridge for stray markers
    rosBridge->AddPublisherFromCommandRead<std::vector<vct3>, sensor_msgs::PointCloud>
        ("Controller", "GetThreeDFiducialPosition",
         "/force_dimension/fiducials");

    // add the bridge after all interfaces have been created
    componentManager->AddComponent(rosBridge);

    // connect all interfaces for the ROS bridge
    for (size_t tool = 0; tool < tracker->GetNumberOfTools(); tool++) {
        toolName = tracker->GetToolName(tool);
        componentManager->Connect(rosBridge->GetName(), toolName,
                                  tracker->GetName(), toolName);
    }
    componentManager->Connect(rosBridge->GetName(), "Controller",
                              tracker->GetName(), "Controller");

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
