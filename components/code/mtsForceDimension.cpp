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

#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstParameterTypes/prmVelocityCartesianGet.h>
#include <cisstParameterTypes/prmForceCartesianGet.h>
#include <cisstParameterTypes/prmForceCartesianSet.h>
#include <cisstParameterTypes/prmForceTorqueJointSet.h>
#include <cisstParameterTypes/prmStateJoint.h>

#include <json/json.h> // in order to read config file

#include <drdc.h>
#include <dhdc.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsForceDimension, mtsTaskContinuous, mtsTaskContinuousConstructorArg);

class mtsForceDimensionDevice
{
public:
    mtsForceDimensionDevice(const char deviceId,
                            const std::string & name,
                            mtsStateTable * stateTable,
                            mtsInterfaceProvided * interfaceProvided);
    void Startup(void);
    void Run(void);
    void Cleanup(void);
    inline const std::string & Name(void) const {
        return mName;
    }

protected:
    void GetRobotData(void);
    void SetControlMode(const mtsForceDimension::ControlModeType & mode);

    void SetRobotControlState(const std::string & state);
    void GetRobotControlState(std::string & state) const;
    std::string mArmState;

    void SetPositionCartesian(const prmForceCartesianSet & desiredForceTorque);
    void SetPositionGoalCartesian(const prmPositionCartesianSet & newPosition);
    void SetWrenchBody(const prmForceCartesianSet & newForce);
    void SetEffortGripper(const prmForceTorqueJointSet & effortGripper);
    void SetGravityCompensation(const bool & gravityCompensation);
    void LockOrientation(const vctMatRot3 & orientation);
    void UnlockOrientation(void);
    void Freeze(void);

    struct {
        mtsFunctionWrite RobotState;
    } MessageEvents;

    char mDeviceId;
    std::string mName;
    mtsStateTable * mStateTable;
    mtsInterfaceProvided * mInterface;
    std::string mSystemName;

    double mGripperDirection;

    prmPositionCartesianGet mPositionCartesian;
    prmVelocityCartesianGet mVelocityCartesian;
    prmForceCartesianGet mForceTorqueCartesian;

    prmStateJoint mStateGripper;
    vctMatRot3 mRotationOffset, mRawOrientation;

    mtsForceDimension::ControlModeType mControlMode;

    bool mNewPositionGoal;
    prmPositionCartesianSet mDesiredPosition;
    prmForceCartesianSet mDesiredWrench;
    double mDesiredEffortGripper;
};

mtsForceDimensionDevice::mtsForceDimensionDevice(const char deviceId,
                                                 const std::string & name,
                                                 mtsStateTable * stateTable,
                                                 mtsInterfaceProvided * interfaceProvided):
    mDeviceId(deviceId),
    mName(name),
    mStateTable(stateTable),
    mInterface(interfaceProvided)
{
    mArmState = "DVRK_POSITION_GOAL_CARTESIAN";
    mNewPositionGoal = false;
    mControlMode = mtsForceDimension::UNDEFINED;

    mDesiredWrench.Force().SetAll(0.0);
    mDesiredEffortGripper = 0.0;
    mGripperDirection = 1.0;

    mStateTable->SetAutomaticAdvance(false);
    mStateTable->AddData(mPositionCartesian, "PositionCartesian");
    mStateTable->AddData(mVelocityCartesian, "VelocityCartesian");
    mStateTable->AddData(mForceTorqueCartesian, "ForceTorqueCartesian");
    mStateGripper.Position().SetSize(1);
    mStateTable->AddData(mStateGripper, "StateGripper");

    if (mInterface) {
        mInterface->AddMessageEvents();
        mInterface->AddCommandReadState(*mStateTable, mPositionCartesian,
                                        "GetPositionCartesian");
        mInterface->AddCommandReadState(*mStateTable, mPositionCartesian,
                                        "GetPositionCartesianDesired");
        mInterface->AddCommandReadState(*mStateTable, mVelocityCartesian,
                                        "GetVelocityCartesian");
        mInterface->AddCommandReadState(*mStateTable, mForceTorqueCartesian,
                                        "GetWrenchBody");
        mInterface->AddCommandReadState(*mStateTable, mStateGripper,
                                        "GetStateGripper");

        mInterface->AddCommandWrite(&mtsForceDimensionDevice::SetPositionGoalCartesian,
                                    this, "SetPositionGoalCartesian");
        mInterface->AddCommandWrite(&mtsForceDimensionDevice::SetWrenchBody,
                                    this, "SetWrenchBody");
        mInterface->AddCommandWrite(&mtsForceDimensionDevice::SetEffortGripper,
                                    this, "SetEffortGripper");
        mInterface->AddCommandWrite(&mtsForceDimensionDevice::SetGravityCompensation,
                                    this, "SetGravityCompensation");
        mInterface->AddCommandWrite(&mtsForceDimensionDevice::LockOrientation,
                                    this, "LockOrientation");
        mInterface->AddCommandVoid(&mtsForceDimensionDevice::UnlockOrientation,
                                   this, "UnlockOrientation");
        mInterface->AddCommandVoid(&mtsForceDimensionDevice::Freeze,
                                   this, "Freeze");

        // robot State
        mInterface->AddCommandWrite(&mtsForceDimensionDevice::SetRobotControlState,
                                    this, "SetRobotControlState", std::string(""));
        mInterface->AddCommandRead(&mtsForceDimensionDevice::GetRobotControlState,
                                   this, "GetRobotControlState", std::string(""));

        // events
        mInterface->AddEventWrite(MessageEvents.RobotState, "RobotState", std::string(""));

        // stats
        mInterface->AddCommandReadState(*mStateTable, mStateTable->PeriodStats,
                                        "GetPeriodStatistics");
    }
}


void mtsForceDimensionDevice::Startup(void)
{
    // required to change asynchronous operation mode
    dhdEnableExpertMode();

    if (drdCheckInit(mDeviceId) < 0) {
        drdStop();
        // message = this->GetName() + ": device initialization check failed, drdCheckInit returned: " + dhdErrorGetLastStr();
        //        mInterface->SendError(message);
    }

    if (!drdIsInitialized(mDeviceId)) {
        if (drdAutoInit(mDeviceId) < 0) {
            // message = this->GetName() + ": failed to auto init, last reported error is: "
            //     + dhdErrorGetLastStr();
        }
    }

    // set gripper direction
    if (dhdIsLeftHanded(mDeviceId)) {
        mGripperDirection = -1.0;
    }

    drdStop(true);

    // update current state
    mStateTable->Start();
    GetRobotData();
    SetControlMode(mtsForceDimension::CARTESIAN_POSITION);
    mStateTable->Advance();
}

void mtsForceDimensionDevice::Run(void)
{
    mStateTable->Start();

    // process mts commands
    mInterface->ProcessMailBoxes();

    // get robot data
    GetRobotData();

    // control mode
    switch (mControlMode) {
    case mtsForceDimension::CARTESIAN_EFFORT:
        dhdSetForceAndTorqueAndGripperForce(mDesiredWrench.Force()[0],
                                            mDesiredWrench.Force()[1],
                                            mDesiredWrench.Force()[2],
                                            mDesiredWrench.Force()[3],
                                            mDesiredWrench.Force()[4],
                                            mDesiredWrench.Force()[5],
                                            mGripperDirection * mDesiredEffortGripper,
                                            mDeviceId);
        break;
    case mtsForceDimension::CARTESIAN_POSITION:
        if (mNewPositionGoal) {
            drdMoveToPos(mDesiredPosition.Goal().Translation().X(),
                         mDesiredPosition.Goal().Translation().Y(),
                         mDesiredPosition.Goal().Translation().Z(),
                         false,
                         mDeviceId);
            mNewPositionGoal = false;
        }
        break;
    default:
        break;
    }
    mStateTable->Advance();
}

void mtsForceDimensionDevice::Cleanup(void)
{
    dhdClose(mDeviceId);
}

void mtsForceDimensionDevice::GetRobotData(void)
{
    double rotation[3][3];
    // position
    dhdGetPositionAndOrientationFrame(&mPositionCartesian.Position().Translation().X(),
                                      &mPositionCartesian.Position().Translation().Y(),
                                      &mPositionCartesian.Position().Translation().Z(),
                                      rotation,
                                      mDeviceId);
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
                         &mVelocityCartesian.VelocityLinear().Z(),
                         mDeviceId);
    dhdGetAngularVelocityRad(&mVelocityCartesian.VelocityAngular().X(),
                             &mVelocityCartesian.VelocityAngular().Y(),
                             &mVelocityCartesian.VelocityAngular().Z(),
                             mDeviceId);
    mVelocityCartesian.SetValid(true);

    // force
    dhdGetForceAndTorque(&mForceTorqueCartesian.Force()[0],
                         &mForceTorqueCartesian.Force()[1],
                         &mForceTorqueCartesian.Force()[2],
                         &mForceTorqueCartesian.Force()[3],
                         &mForceTorqueCartesian.Force()[4],
                         &mForceTorqueCartesian.Force()[5],
                         mDeviceId);
    mForceTorqueCartesian.SetValid(true);

    // gripper
    dhdGetGripperAngleRad(&mStateGripper.Position().at(0), mDeviceId);
    mStateGripper.Position().at(0) *= mGripperDirection;
}

void mtsForceDimensionDevice::SetRobotControlState(const std::string & state)
{
    mArmState = state;
    MessageEvents.RobotState(state);
    mInterface->SendStatus(mInterface->GetName() + ": state is now " + state);
}

void mtsForceDimensionDevice::GetRobotControlState(std::string & state) const
{
    state = mArmState;
}

void mtsForceDimensionDevice::SetControlMode(const mtsForceDimension::ControlModeType & mode)
{
    // return if we are already in this mode
    if (mode == mControlMode) {
        return;
    }
    // transition to new mode
    switch (mode) {
    case mtsForceDimension::CARTESIAN_POSITION:
        mNewPositionGoal = false;
        drdRegulatePos(true, mDeviceId);
        drdRegulateRot(false, mDeviceId);
        drdRegulateGrip(false, mDeviceId);
        drdStart();
        // start from current position
        mDesiredPosition.Goal().Assign(mPositionCartesian.Position());
        mNewPositionGoal = true;
        break;
    case mtsForceDimension::CARTESIAN_EFFORT:
        drdRegulatePos(false, mDeviceId);
        drdStop(true, mDeviceId);
        // start with 0 forces
        mDesiredWrench.Force().SetAll(0.0);
        mDesiredEffortGripper = 0.0;
        break;
    default:
        break;
    }
    // assign mode
    mControlMode = mode;
}

void mtsForceDimensionDevice::SetWrenchBody(const prmForceCartesianSet & wrench)
{
    SetControlMode(mtsForceDimension::CARTESIAN_EFFORT);
    mDesiredWrench = wrench;
}

void mtsForceDimensionDevice::SetEffortGripper(const prmForceTorqueJointSet & effortGripper)
{
    SetControlMode(mtsForceDimension::CARTESIAN_EFFORT);
    mDesiredEffortGripper = effortGripper.ForceTorque().at(0) * mGripperDirection;
}

void mtsForceDimensionDevice::SetPositionGoalCartesian(const prmPositionCartesianSet & position)
{
    SetControlMode(mtsForceDimension::CARTESIAN_POSITION);
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

void mtsForceDimensionDevice::UnlockOrientation(void)
{
    // mRotationOffset = vctMatRot3();
}

void mtsForceDimensionDevice::Freeze(void)
{
}

void mtsForceDimensionDevice::LockOrientation(const vctMatRot3 & orientation)
{
    mRotationOffset = orientation * mRawOrientation.Inverse();
}

void mtsForceDimensionDevice::SetGravityCompensation(const bool & gravity)
{
    if (gravity) {
        dhdSetGravityCompensation(DHD_ON, mDeviceId);
    } else {
        dhdSetGravityCompensation(DHD_OFF, mDeviceId);
    }
}

void mtsForceDimension::Init(void)
{
    mConfigured = false;
}

void mtsForceDimension::Configure(const std::string & filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: using " << filename << std::endl;

    if (mConfigured) {
        CMN_LOG_CLASS_INIT_VERBOSE << "Configure: already configured" << std::endl;
        return;
    }

    typedef std::list<std::string> NoSerialDevicesType;
    NoSerialDevicesType noSerialDevices;
    typedef std::map<int, std::string> SerialDevicesType;
    SerialDevicesType serialDevices;

    if (filename != "") {
        // read JSON file passed as param, see configAtracsysFusionTrack.json for an example
        std::ifstream jsonStream;
        jsonStream.open(filename.c_str());

        Json::Value jsonConfig, jsonValue;
        Json::Reader jsonReader;
        if (!jsonReader.parse(jsonStream, jsonConfig)) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to parse configuration" << std::endl
                                     << jsonReader.getFormattedErrorMessages();
            return;
        }

        const Json::Value jsonDevices = jsonConfig["devices"];
        for (unsigned int index = 0; index < jsonDevices.size(); ++index) {
            jsonValue = jsonDevices[index];
            std::string deviceName = jsonValue["name"].asString();
            // make sure toolName is valid
            if (deviceName == "") {
                CMN_LOG_CLASS_INIT_ERROR << "Configure: invalid tool["
                                         << index << "] name found in "
                                         << filename << std::endl;
                return;
            }
            if (jsonValue["serial"].empty() || !jsonValue["serial"].isInt()) {
                CMN_LOG_CLASS_INIT_ERROR << "Configure: invalid tool["
                                         << index << "] serial found in "
                                         << filename << " (must be an integer)"
                                         << std::endl;
                return;
            }
            int deviceSerial = jsonValue["serial"].asInt();
            if (deviceSerial == 0) {
                noSerialDevices.push_back(deviceName);
            } else {
                // check that this serial number hasn't already been assigned
                SerialDevicesType::const_iterator found = serialDevices.find(deviceSerial);
                if (found == serialDevices.end()) {
                    serialDevices[deviceSerial] = deviceName;
                } else {
                    CMN_LOG_CLASS_INIT_ERROR << "Configure: invalid tool["
                                             << index << "], serial number "
                                             << deviceSerial << " already used in "
                                             << filename << std::endl;
                    return;
                }
            }
        }
    }

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

    mNumberOfDevices = dhdGetDeviceCount();
    if (mNumberOfDevices < 1) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: can't find any device.  Make sure you have the correct permissions to access the USB devices.  See also README.md for sawForceDimensionSDK" << std::endl;
        return;
    }

    // identify and name each device
    for (int i = 0;
         i < mNumberOfDevices; i++) {
        const int deviceId = dhdOpenID(i);
        if (deviceId == -1) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to open device "
                                     << i << ", id: " << deviceId << std::endl;

        }
        std::string systemName = dhdGetSystemName(deviceId);
        bool hasSerial = true;
        ushort serialUShort = 0;
        if (dhdGetSerialNumber(&serialUShort, deviceId) != 0) {
            // error while getting serial number
            hasSerial = false;
            CMN_LOG_CLASS_INIT_WARNING << "Configure: can't retrieve serial number for device "
                                       << i << ", id: " << deviceId << std::endl;
        }
        // for some reason, FALCON serial number is always 65535
        if (dhdGetSystemType(deviceId) == DHD_DEVICE_FALCON) {
            hasSerial = false;
        }
        // find name of device based on serial number in configuration file
        std::string deviceName = "";
        if (hasSerial) {
            // look for name on map from configuration file
            SerialDevicesType::const_iterator found = serialDevices.find(serialUShort);
            if (found != serialDevices.end()) {
                deviceName = found->second;
            }
        }
        // if we still don't have a name
        if (deviceName == "") {
            // look in the list of names from configuration file
            if (!noSerialDevices.empty()) {
                deviceName = *(noSerialDevices.begin());
                noSerialDevices.pop_front();
            } else {
                // now we just make up a name
                std::stringstream tempName;
                tempName << systemName << std::setfill('0') << std::setw(2) << deviceId;
                deviceName = tempName.str();
            }
        }
        // create the device data and add to list of devices
        mtsStateTable * stateTable
            = new mtsStateTable(StateTable.GetHistoryLength(),
                                deviceName);
        mtsInterfaceProvided * interfaceProvided
            = this->AddInterfaceProvided(deviceName);
        if (!interfaceProvided) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: can't create interface provided with name \""
                                     << deviceName << "\".  Device "
                                     << i << ", id: " << deviceId << std::endl;
            return;
        }
        mtsForceDimensionDevice * device =
            new mtsForceDimensionDevice(deviceId, deviceName,
                                        stateTable, interfaceProvided);
        mDevices.push_back(device);
    }

    #if 0
    // open the first available device
    if (drdOpen() < 0) {
        CMN_LOG_CLASS_INIT_ERROR << this->GetName() << ": can't open device, drdOpen returned: "
                                 << dhdErrorGetLastStr() << std::endl;
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

    drdStop(true);
    #endif
}


void mtsForceDimension::Startup(void)
{
    const DevicesType::iterator end = mDevices.end();
    DevicesType::iterator device;
    for (device = mDevices.begin();
         device != end;
         ++device) {
        (*device)->Startup();
    }
}


std::list<std::string> mtsForceDimension::GetDeviceNames(void) const
{
    std::list<std::string> result;
    const DevicesType::const_iterator end = mDevices.end();
    DevicesType::const_iterator device;
    for (device = mDevices.begin();
         device != end;
         ++device) {
        result.push_back((*device)->Name());
    }
    return result;
}


void mtsForceDimension::Run(void)
{
    // process mts commands using interface->ProcessMailBoxes
    // DO NOT USE ProcessQueuedCommands() !!!

    const DevicesType::iterator end = mDevices.end();
    DevicesType::iterator device;
    for (device = mDevices.begin();
         device != end;
         ++device) {
        (*device)->Run();
    }
}

void mtsForceDimension::Cleanup(void)
{
    const DevicesType::iterator end = mDevices.end();
    DevicesType::iterator device;
    for (device = mDevices.begin();
         device != end;
         ++device) {
        (*device)->Cleanup();
    }
    drdClose();
}
