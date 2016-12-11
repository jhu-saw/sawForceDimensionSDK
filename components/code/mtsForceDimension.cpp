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

#include <cisstMultiTask/mtsInterfaceProvided.h>

#include <sawForceDimensionSDK/mtsForceDimension.h>

#include <drdc.h>
#include <dhdc.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsForceDimension, mtsTaskContinuous, mtsTaskContinuousConstructorArg);

void mtsForceDimension::Init(void)
{
    mNumberOfDevices = 0;


    StateTable.AddData(mPositionCartesian, "PositionCartesian");
    StateTable.AddData(mForceTorqueCartesian, "ForceTorqueCartesian");

    mtsInterfaceProvided * provided = AddInterfaceProvided("Device");
    if (provided) {
        provided->AddCommandReadState(StateTable, mPositionCartesian,
                                      "GetPositionCartesian");
        provided->AddCommandReadState(StateTable, mForceTorqueCartesian,
                                      "GetForceTorqueCartesian");
        provided->AddCommandReadState(StateTable, StateTable.PeriodStats,
                                      "GetPeriodStatistics");
    }
}

void mtsForceDimension::Configure(const std::string & filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: using " << filename << std::endl;

    std::cerr << CMN_LOG_DETAILS << " this needs to be optional, maybe shouldn't autoinit" << std::endl;

    // sdk version number
    dhdGetSDKVersion (&(mSDKVersion.Major),
                      &(mSDKVersion.Minor),
                      &(mSDKVersion.Release),
                      &(mSDKVersion.Revision));
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: using SDK "
                               << mSDKVersion.Major << "."
                               << mSDKVersion.Minor << "."
                               << mSDKVersion.Release << "."
                               << mSDKVersion.Revision << std::endl;
    std::cerr << "Configure: using SDK "
              << mSDKVersion.Major << "."
              << mSDKVersion.Minor << "."
              << mSDKVersion.Release << "."
              << mSDKVersion.Revision << std::endl;

    // required to change asynchronous operation mode
    dhdEnableExpertMode();

    // open the first available device
    if (drdOpen() < 0) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: can't open device, drdOpen returned: "
                                 << dhdErrorGetLastStr()
                                 << std::endl;
        return;
    }

    // identify device
    mSystemName = dhdGetSystemName();
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: found system " << mSystemName << std::endl;

    mNumberOfDevices = dhdGetDeviceCount();
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: number of devices " << mNumberOfDevices << std::endl;

    if (!drdIsInitialized()) {
        if (drdAutoInit() < 0) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: auto init failed: "
                                     << dhdErrorGetLastStr()
                                     << std::endl;
        }
    }

        // dhdEnableForce(DHD_ON);
        // dhdSetGravityCompensation(DHD_ON);
}


void mtsForceDimension::Startup(void)
{
    CMN_LOG_CLASS_RUN_ERROR << "Startup" << std::endl;
}


void mtsForceDimension::Run(void)
{
    // process mts commands
    ProcessQueuedCommands();

    // replace by loop to handle multiple devices
    double rotation[3][3];
    if (mNumberOfDevices > 0) {
        dhdGetPositionAndOrientationFrame(&mPositionCartesian.Position().Translation().X(),
                                          &mPositionCartesian.Position().Translation().Y(),
                                          &mPositionCartesian.Position().Translation().Z(),
                                          rotation);
    }
}

void mtsForceDimension::Cleanup(void)
{
    drdClose();
}
