/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2014-07-17

  (C) Copyright 2014-2017 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsForceDimension_h
#define _mtsForceDimension_h

#include <cisstCommon/cmnPath.h>
#include <cisstMultiTask/mtsTaskContinuous.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstParameterTypes/prmVelocityCartesianGet.h>
#include <cisstParameterTypes/prmForceCartesianGet.h>
#include <cisstParameterTypes/prmForceCartesianSet.h>
#include <cisstParameterTypes/prmStateJoint.h>

#include <json/json.h> // in order to read config file

#include <sawForceDimensionSDK/sawForceDimensionSDKExport.h>  // always include last

class CISST_EXPORT mtsForceDimension: public mtsTaskContinuous
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

 public:
    inline mtsForceDimension(const std::string & componentName):
        mtsTaskContinuous(componentName, 256) {
        Init();
    }

    inline mtsForceDimension(const mtsTaskContinuousConstructorArg & arg):
        mtsTaskContinuous(arg) {
        Init();
    }

    ~mtsForceDimension(void) {};

    void Configure(const std::string & filename = "");
    void Startup(void);
    void Run(void);
    void Cleanup(void);

protected:
    enum ControlModeType {UNDEFINED, CARTESIAN_POSITION, CARTESIAN_EFFORT};

    /*! Code called by all constructors. */
    void Init(void);
    void GetRobotData(void);
    void SetControlMode(const ControlModeType & mode);

    void SetRobotControlState(const std::string & state);
    void GetRobotControlState(std::string & state) const;
    std::string mArmState;

    void SetPositionCartesian(const prmForceCartesianSet & desiredForceTorque);
    void SetPositionGoalCartesian(const prmPositionCartesianSet & newPosition);
    void SetWrenchBody(const prmForceCartesianSet & newForce);
    void SetGravityCompensation(const bool & gravityCompensation);
    void LockOrientation(const vctMatRot3 & orientation);
    void UnlockOrientation(void);
    void Freeze(void);

    struct {
        mtsFunctionWrite RobotState;
    } MessageEvents;
    mtsInterfaceProvided * mInterface;

    struct {
        int Major;
        int Minor;
        int Release;
        int Revision;
    } mSDKVersion;

    int mNumberOfDevices;
    char mDeviceId;
    std::string mSystemName;

    double mGripperDirection;

    prmPositionCartesianGet mPositionCartesian;
    prmVelocityCartesianGet mVelocityCartesian;
    prmForceCartesianGet mForceTorqueCartesian;

    prmStateJoint mStateGripper;
    vctMatRot3 mRotationOffset, mRawOrientation;

    ControlModeType mControlMode;

    bool mNewPositionGoal;
    prmPositionCartesianSet mDesiredPosition;
    prmForceCartesianSet mDesiredWrench;
    double mDesiredEffortGripper;    
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsForceDimension);

#endif  // _mtsForceDimension_h
