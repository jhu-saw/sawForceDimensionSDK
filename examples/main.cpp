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

    options.AddOptionOneValue("j", "json-config",
                              "json configuration file",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &jsonConfigFile);

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
    for (size_t tool = 0; tool < tracker->GetNumberOfTools(); tool++) {
        toolName = tracker->GetToolName(tool);
        toolWidget = new mtsForceDimensionSDKToolQtWidget(toolName + "-GUI");
        toolWidget->Configure();
        componentManager->AddComponent(toolWidget);
        componentManager->Connect(toolWidget->GetName(), "Tool",
                                  tracker->GetName(), toolName);
        tabWidget->addTab(toolWidget, toolName.c_str());
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
