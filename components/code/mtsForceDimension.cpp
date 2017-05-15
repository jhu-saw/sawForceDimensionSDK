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
    mNewPositionGoal = false;
    mControlMode = UNDEFINED;

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

    // update current state
    GetRobotData();

    // freeze in position
    SetControlMode(CARTESIAN_POSITION);
}


void mtsForceDimension::GetRobotData(void)
{
    double rotation[3][3];
    // to be replaced by loop to handle multiple devices
    if (mNumberOfDevices > 0) {
        // position
        dhdGetPositionAndOrientationFrame(&mPositionCartesian.Position().Translation().X(),
                                          &mPositionCartesian.Position().Translation().Y(),
                                          &mPositionCartesian.Position().Translation().Z(),
                                          rotation);
        mRawOrientation.Row(0).Assign(rotation[0]);
        mRawOrientation.Row(1).Assign(rotation[1]);
        mRawOrientation.Row(2).Assign(rotation[2]);
        // apply rotation to tip, this should come from a configuration file
        vctMatRot3 rotationTip;
        rotationTip.Assign( 0.0,  0.0, -1.0,
                            0.0, -1.0,  0.0,
                            -1.0, 0.0,  0.0);
        mRawOrientation = mRawOrientation * rotationTip;
        mPositionCartesian.Position().Rotation() = mRotationOffset * mRawOrientation;
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
    }
}

void mtsForceDimension::Run(void)
{
    // process mts commands
    ProcessQueuedCommands();

    // get robot data
    GetRobotData();

    // replace by loop to handle multiple devices
    if (mNumberOfDevices > 0) {
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
            if (mNewPositionGoal) {
                drdMoveToPos(mDesiredPosition.Goal().Translation().X(),
                             mDesiredPosition.Goal().Translation().Y(),
                             mDesiredPosition.Goal().Translation().Z(),
                             false);
                mNewPositionGoal = false;
            }
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

void mtsForceDimension::SetControlMode(const ControlModeType & mode)
{
    // return if we are already in this mode
    if (mode == mControlMode) {
        return;
    }
    // transition to new mode
    switch (mode) {
    case CARTESIAN_POSITION:
        mNewPositionGoal = false;
        drdRegulatePos(true);
        drdRegulateRot(false);
        drdRegulateGrip(false);
        drdStart();
        // start from current position
        mDesiredPosition.Goal().Assign(mPositionCartesian.Position());
        mNewPositionGoal = true;
        break;
    case CARTESIAN_EFFORT:
        drdRegulatePos(false);
        drdStop(true);
        // start with 0 forces
        mDesiredWrench.Force().SetAll(0.0);
        break;
    default:
        break;
    }
    // assign mode
    mControlMode = mode;
}

void mtsForceDimension::SetWrenchBody(const prmForceCartesianSet & wrench)
{
    SetControlMode(CARTESIAN_EFFORT);
    mDesiredWrench = wrench;
}

void mtsForceDimension::SetPositionGoalCartesian(const prmPositionCartesianSet & position)
{
    SetControlMode(CARTESIAN_POSITION);
    mDesiredPosition = position;
    mNewPositionGoal = true;

    // Rc: rotation current
    // Rd: rotation desired
    // Ro: rotation offset to make the rotation looks like desired
    // Ro * Rc = Rd
    // Ro = Rd * Rc_transpose
    mRotationOffset = position.Goal().Rotation() // desired
        * mRawOrientation.Inverse();
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
    mRotationOffset = orientation * mRawOrientation.Inverse();
}

void mtsForceDimension::SetGravityCompensation(const bool & gravity)
{
    if (gravity) {
        dhdSetGravityCompensation(DHD_ON);
    } else {
        dhdSetGravityCompensation(DHD_OFF);
    }
}
