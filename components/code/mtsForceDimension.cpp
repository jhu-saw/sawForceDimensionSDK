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

#include <cisstMultiTask/mtsInterfaceProvided.h>

#include <sawForceDimensionSDK/mtsForceDimension.h>

#include <drdc.h>
#include <dhdc.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsForceDimension, mtsTaskContinuous, mtsTaskContinuousConstructorArg);

void mtsForceDimension::Init(void)
{
    mArmState = "DVRK_POSITION_GOAL_CARTESIAN";
    mControlMode = CARTESIAN_POSITION;

    mDesiredWrench.Force().SetAll(0.0);
    mDesiredEffortGripper = 0.0;
    mGripperDirection = 1.0;

    mNumberOfDevices = 0;

    StateTable.AddData(mPositionCartesian, "PositionCartesian");
    StateTable.AddData(mVelocityCartesian, "VelocityCartesian");
    StateTable.AddData(mForceTorqueCartesian, "ForceTorqueCartesian");
    mStateGripper.Position().SetSize(1);
    StateTable.AddData(mStateGripper, "StateGripper");

    mInterface = AddInterfaceProvided("Robot");
    if (mInterface) {
        mInterface->AddMessageEvents();
        mInterface->AddCommandReadState(StateTable, mPositionCartesian,
                                        "GetPositionCartesian");
        mInterface->AddCommandReadState(StateTable, mPositionCartesian,
                                        "GetPositionCartesianDesired");
        mInterface->AddCommandReadState(StateTable, mVelocityCartesian,
                                        "GetVelocityCartesian");
        mInterface->AddCommandReadState(StateTable, mForceTorqueCartesian,
                                        "GetWrenchBody");
        mInterface->AddCommandReadState(StateTable, mStateGripper,
                                        "GetStateGripper");

        mInterface->AddCommandWrite(&mtsForceDimension::SetPositionGoalCartesian,
                                    this, "SetPositionGoalCartesian");
        mInterface->AddCommandWrite(&mtsForceDimension::SetWrenchBody,
                                    this, "SetWrenchBody");
        mInterface->AddCommandWrite(&mtsForceDimension::SetGravityCompensation,
                                    this, "SetGravityCompensation");
        mInterface->AddCommandWrite(&mtsForceDimension::LockOrientation,
                                    this, "LockOrientation");
        mInterface->AddCommandVoid(&mtsForceDimension::UnlockOrientation,
                                   this, "UnlockOrientation");
        mInterface->AddCommandVoid(&mtsForceDimension::Freeze,
                                   this, "Freeze");

        // robot State
        mInterface->AddCommandWrite(&mtsForceDimension::SetRobotControlState,
                                    this, "SetRobotControlState", std::string(""));
        mInterface->AddCommandRead(&mtsForceDimension::GetRobotControlState,
                                   this, "GetRobotControlState", std::string(""));

        // events
        mInterface->AddEventWrite(MessageEvents.RobotState, "RobotState", std::string(""));

        // stats
        mInterface->AddCommandReadState(StateTable, StateTable.PeriodStats,
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
        mInterface->SendError(message);
        return;
    }

    if (drdCheckInit() < 0) {
       drdStop();
       message = this->GetName() + ": device initialization check failed, drdCheckInit returned: " + dhdErrorGetLastStr();
       mInterface->SendError(message);
    }

    // identify device
    mSystemName = dhdGetSystemName();
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: found system " << mSystemName << std::endl;

    mNumberOfDevices = dhdGetDeviceCount();
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: number of devices " << mNumberOfDevices << std::endl;

    if (!drdIsInitialized()) {
        if (drdAutoInit() < 0) {
            message = this->GetName() + ": failed to auto init, last reported error is: "
                + dhdErrorGetLastStr();
        }
    }

    // set gripper direction
    if (dhdIsLeftHanded()) {
        mGripperDirection = -1.0;
    }
}


void mtsForceDimension::Startup(void)
{
    CMN_LOG_CLASS_RUN_ERROR << "Startup" << std::endl;
    drdStop(true);
}


void mtsForceDimension::Run(void)
{
    // process mts commands
    ProcessQueuedCommands();

    // replace by loop to handle multiple devices
    double rotation[3][3];
    if (mNumberOfDevices > 0) {
        // position
        dhdGetPositionAndOrientationFrame(&mPositionCartesian.Position().Translation().X(),
                                          &mPositionCartesian.Position().Translation().Y(),
                                          &mPositionCartesian.Position().Translation().Z(),
                                          rotation);
        mRawOrientation.Row(0).Assign(rotation[0]);
        mRawOrientation.Row(1).Assign(rotation[1]);
        mRawOrientation.Row(2).Assign(rotation[2]);
        mPositionCartesian.Position().Rotation() = mRotationOffset;
        //        mPositionCartesian.Position().Rotation() = mRawOrientation * mRotationOffset;
        mPositionCartesian.Valid() = true;

        // velocity
        dhdGetLinearVelocity(&mVelocityCartesian.VelocityLinear().X(),
                             &mVelocityCartesian.VelocityLinear().Y(),
                             &mVelocityCartesian.VelocityLinear().Z());
        dhdGetAngularVelocityRad(&mVelocityCartesian.VelocityAngular().X(),
                                 &mVelocityCartesian.VelocityAngular().Y(),
                                 &mVelocityCartesian.VelocityAngular().Z());
        mVelocityCartesian.SetValid(true);

        // force
        dhdGetForceAndTorque(&mForceTorqueCartesian.Force()[0],
                             &mForceTorqueCartesian.Force()[1],
                             &mForceTorqueCartesian.Force()[2],
                             &mForceTorqueCartesian.Force()[3],
                             &mForceTorqueCartesian.Force()[4],
                             &mForceTorqueCartesian.Force()[5]);
        mForceTorqueCartesian.SetValid(true);

        // gripper
        dhdGetGripperAngleRad(&mStateGripper.Position().at(0));
        mStateGripper.Position().at(0) *= mGripperDirection;

        // control mode
        switch (mControlMode) {
        case CARTESIAN_EFFORT:
            dhdSetForceAndTorqueAndGripperForce(mDesiredWrench.Force()[0],
                                                mDesiredWrench.Force()[1],
                                                mDesiredWrench.Force()[2],
                                                mDesiredWrench.Force()[3],
                                                mDesiredWrench.Force()[4],
                                                mDesiredWrench.Force()[5],
                                                mGripperDirection * mDesiredEffortGripper);
            break;
        case CARTESIAN_POSITION:
            break;
        default:
            break;
        }
    }
}

void mtsForceDimension::Cleanup(void)
{
    drdClose();
}

void mtsForceDimension::SetRobotControlState(const std::string & state)
{
    mArmState = state;
    MessageEvents.RobotState(state);
    mInterface->SendStatus(this->GetName() + ": state is now " + state);
}

void mtsForceDimension::GetRobotControlState(std::string & state) const
{
    state = mArmState;
}

void mtsForceDimension::SetWrenchBody(const prmForceCartesianSet & wrench)
{
    mControlMode = CARTESIAN_EFFORT;
    mDesiredWrench = wrench;
}

void mtsForceDimension::SetPositionGoalCartesian(const prmPositionCartesianSet & position)
{
    mRotationOffset = position.Goal().Rotation();
    //     mRotationOffset = mRawOrientation.Inverse() * position.Goal().Rotation();
    std::cerr << "LockOrientation" << std::endl
              << "Current: " << std::endl
              << mRawOrientation << std::endl
              << "Desired: " << std::endl
              << position.Goal().Rotation() << std::endl
              << "Offset: " << std::endl
              << mRotationOffset << std::endl;
}

void mtsForceDimension::UnlockOrientation(void)
{
    // mRotationOffset = vctMatRot3();
}

void mtsForceDimension::Freeze(void)
{
}

void mtsForceDimension::LockOrientation(const vctMatRot3 & orientation)
{
}

void mtsForceDimension::SetGravityCompensation(const bool & gravity)
{
    if (gravity) {
        dhdSetForceAndTorqueAndGripperForce(0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0);
    } else {

    }
}
