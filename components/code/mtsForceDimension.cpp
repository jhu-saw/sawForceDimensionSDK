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
    mArmState = "DVRK_POSITION_GOAL_CARTESIAN";
    
    mNumberOfDevices = 0;

    StateTable.AddData(mArmState, "ArmState");
    StateTable.AddData(mPositionCartesian, "PositionCartesian");
    StateTable.AddData(mVelocityCartesian, "VelocityCartesian");
    StateTable.AddData(mForceTorqueCartesian, "ForceTorqueCartesian");
    StateTable.AddData(mPositionGripper, "PositionGripper");
    
    mtsInterfaceProvided * provided = AddInterfaceProvided("Device");
    if (provided) {
        provided->AddCommandReadState(StateTable, mPositionCartesian,
                                      "GetPositionCartesian");
        provided->AddCommandReadState(StateTable, mPositionCartesian,
                                      "GetPositionCartesianDesired");
        provided->AddCommandReadState(StateTable, mVelocityCartesian,
                                      "GetVelocityCartesian");
        provided->AddCommandReadState(StateTable, mForceTorqueCartesian,
                                      "GetForceTorqueCartesian");
        provided->AddCommandReadState(StateTable, mPositionGripper,
                                      "GetGripperPosition");

        provided->AddCommandWrite(&mtsForceDimension::SetPositionGoalCartesian,
                                  this, "SetPositionGoalCartesian");
        provided->AddCommandWrite(&mtsForceDimension::SetWrenchBody,
                                  this, "SetWrenchBody");
        provided->AddCommandWrite(&mtsForceDimension::SetGravityCompensation,
                                  this, "SetGravityCompensation");
        provided->AddCommandWrite(&mtsForceDimension::LockOrientation,
                                  this, "LockOrientation");
        provided->AddCommandVoid(&mtsForceDimension::UnlockOrientation,
                                 this, "UnlockOrientation");

        // robot State
        provided->AddCommandWrite(&mtsForceDimension::SetRobotControlState,
                                  this, "SetRobotControlState", std::string(""));
        provided->AddCommandReadState(StateTable, mArmState,
                                      "GetRobotControlState");
        
        // human readable messages
        provided->AddEventWrite(MessageEvents.Status, "Status", std::string(""));
        provided->AddEventWrite(MessageEvents.Warning, "Warning", std::string(""));
        provided->AddEventWrite(MessageEvents.Error, "Error", std::string(""));
        provided->AddEventWrite(MessageEvents.RobotState, "RobotState", std::string(""));

        // stats
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

    // required to change asynchronous operation mode
    dhdEnableExpertMode();

    std::string message;
    
    // open the first available device
    if (drdOpen() < 0) {
        message = this->GetName() + ": can't open device, drdOpen returned: "
            + dhdErrorGetLastStr();
        CMN_LOG_CLASS_INIT_ERROR << message
                                 << std::endl;
        MessageEvents.Error(message);                            
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

    dhdEnableForce(DHD_ON);
    dhdSetGravityCompensation(DHD_ON);
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
    vctMatRot3 vctRotation;
    if (mNumberOfDevices > 0) {
        // position
        dhdGetPositionAndOrientationFrame(&mPositionCartesian.Position().Translation().X(),
                                          &mPositionCartesian.Position().Translation().Y(),
                                          &mPositionCartesian.Position().Translation().Z(),
                                          rotation);
        vctRotation.Row(0).Assign(rotation[0]);
        vctRotation.Row(1).Assign(rotation[1]);
        vctRotation.Row(2).Assign(rotation[2]);
        mPositionCartesian.Position().Rotation() = mRotationOffset * vctRotation;

        // velocity
        dhdGetLinearVelocity(&mVelocityCartesian.VelocityLinear().X(),
                             &mVelocityCartesian.VelocityLinear().Y(),
                             &mVelocityCartesian.VelocityLinear().Z());
        dhdGetAngularVelocityRad(&mVelocityCartesian.VelocityAngular().X(),
                                 &mVelocityCartesian.VelocityAngular().Y(),
                                 &mVelocityCartesian.VelocityAngular().Z());

        // force
        dhdGetForceAndTorque(&mForceTorqueCartesian.Force()[0],
                             &mForceTorqueCartesian.Force()[1],
                             &mForceTorqueCartesian.Force()[2],
                             &mForceTorqueCartesian.Force()[3],
                             &mForceTorqueCartesian.Force()[4],
                             &mForceTorqueCartesian.Force()[5]);

        // gripper
        dhdGetGripperAngleRad(&mPositionGripper);
    }
}

void mtsForceDimension::Cleanup(void)
{
    drdClose();
}

void mtsForceDimension::SetRobotControlState(const std::string & state)
{
    mArmState = state;
    MessageEvents.RobotState(std::string(state));
}

void mtsForceDimension::SetWrenchBody(const prmForceCartesianSet & wrench)
{
    dhdSetForce(wrench.Force().X(),
                wrench.Force().Y(),
                wrench.Force().Z());
}

void mtsForceDimension::SetPositionGoalCartesian(const prmPositionCartesianSet & position)
{
    std::cerr << CMN_LOG_DETAILS << " SetPositionGoalCartesian not implemented" << std::endl;
}

void mtsForceDimension::UnlockOrientation(void)
{
    mRotationOffset = vctMatRot3();
}

void mtsForceDimension::LockOrientation(const vctMatRot3 & orientation)
{
    mRotationOffset = orientation * mPositionCartesian.Position().Rotation().Inverse();
}

void mtsForceDimension::SetGravityCompensation(const bool & gravity)
{
    if (gravity) {
        dhdSetGravityCompensation(DHD_ON);
    } else {
        dhdSetGravityCompensation(DHD_OFF);
    }
}
