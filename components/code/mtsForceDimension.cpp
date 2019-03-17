/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2016-11-10

  (C) Copyright 2016-2019 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstMultiTask/mtsInterfaceProvided.h>

#include <sawForceDimensionSDK/mtsForceDimension.h>

#include <cisstParameterTypes/prmOperatingState.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstParameterTypes/prmVelocityCartesianGet.h>
#include <cisstParameterTypes/prmForceCartesianGet.h>
#include <cisstParameterTypes/prmForceCartesianSet.h>
#include <cisstParameterTypes/prmForceTorqueJointSet.h>
#include <cisstParameterTypes/prmStateJoint.h>
#include <cisstParameterTypes/prmEventButton.h>

#include <json/json.h> // in order to read config file

#include <drdc.h>
#include <dhdc.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsForceDimension, mtsTaskContinuous, mtsTaskContinuousConstructorArg);

class mtsForceDimensionDevice
{
public:
    typedef std::list<mtsInterfaceProvided *> ButtonInterfaces;

    mtsForceDimensionDevice(const int deviceId,
                            const std::string & name,
                            mtsStateTable * stateTable,
                            mtsInterfaceProvided * interfaceProvided,
                            const ButtonInterfaces & buttonInterfaces);
    void Startup(void);
    void Run(void);
    void Cleanup(void);
    inline const std::string & Name(void) const {
        return m_name;
    }

    void GetButtonNames(std::list<std::string> & result) const;

protected:
    void GetRobotData(void);
    void SetControlMode(const mtsForceDimension::ControlModeType & mode);

    // crtk state
    void state_command(const std::string & command);
    prmOperatingState m_operating_state;
    mtsFunctionWrite m_operating_state_event;

    void servo_cp(const prmPositionCartesianSet & newPosition);
    void servo_cf(const prmForceCartesianSet & newForce);
    void move_cp(const prmPositionCartesianSet & newPosition);
    void gripper_servo_jf(const prmForceTorqueJointSet & effortGripper);
    void SetGravityCompensation(const bool & gravityCompensation);
    void LockOrientation(const vctMatRot3 & orientation);
    void UnlockOrientation(void);
    void Freeze(void);

    int m_device_id;
    std::string m_device_id_string;
    std::string m_name;
    mtsStateTable * m_state_table;
    mtsInterfaceProvided * m_interface;

    uint mPreviousButtonMask;
    struct ButtonData {
        std::string Name;
        mtsFunctionWrite Function;
        bool Pressed;
    };
    typedef std::list<ButtonData *> ButtonsData;

    ButtonsData mButtonCallbacks;

    double m_gripper_direction;

    prmPositionCartesianGet m_measured_cp, m_setpoint_cp;
    prmVelocityCartesianGet m_measured_cv;
    prmForceCartesianGet m_measured_cf;

    prmStateJoint m_gripper_measured_js;
    vctMatRot3 m_rotation_offset, mRawOrientation;

    mtsForceDimension::ControlModeType mControlMode;

    bool m_new_servo_cp;
    prmPositionCartesianSet m_servo_cp;
    prmForceCartesianSet m_servo_cf;
    double m_gripper_servo_jf;
};

mtsForceDimensionDevice::mtsForceDimensionDevice(const int deviceId,
                                                 const std::string & name,
                                                 mtsStateTable * stateTable,
                                                 mtsInterfaceProvided * interfaceProvided,
                                                 const ButtonInterfaces & buttonInterfaces):
    m_device_id(deviceId),
    m_name(name),
    m_state_table(stateTable),
    m_interface(interfaceProvided)
{
    std::stringstream idString;
    idString << deviceId;
    m_device_id_string = idString.str();

    m_operating_state.State() = prmOperatingState::VOID;
    m_operating_state.IsBusy() = false;
    m_operating_state.Valid() = true;

    m_new_servo_cp = false;
    mControlMode = mtsForceDimension::UNDEFINED;

    m_servo_cf.Force().SetAll(0.0);
    m_gripper_servo_jf = 0.0;
    m_gripper_direction = 1.0;

    m_measured_cp.SetReferenceFrame(m_name + "_base");
    m_measured_cp.SetMovingFrame(m_name);
    m_setpoint_cp.SetReferenceFrame(m_name + "_base");
    m_setpoint_cp.SetMovingFrame(m_name);

    m_state_table->SetAutomaticAdvance(false);
    m_state_table->AddData(m_operating_state, "operating_state");
    m_state_table->AddData(m_measured_cp, "measured_cp");
    m_state_table->AddData(m_measured_cv, "measured_cv");
    m_state_table->AddData(m_measured_cf, "measured_cf");
    m_state_table->AddData(m_setpoint_cp, "setpoint_cp");
    m_gripper_measured_js.Position().SetSize(1);
    m_gripper_measured_js.Velocity().SetSize(1);
    m_gripper_measured_js.Effort().SetSize(1);
    m_state_table->AddData(m_gripper_measured_js, "gripper_measured_js");

    if (m_interface) {
        m_interface->AddMessageEvents();
        m_interface->AddCommandReadState(*m_state_table, m_measured_cp,
                                         "measured_cp");
        m_interface->AddCommandReadState(*m_state_table, m_measured_cv,
                                         "measured_cv");
        m_interface->AddCommandReadState(*m_state_table, m_measured_cf,
                                         "measured_cf");
        m_interface->AddCommandReadState(*m_state_table, m_gripper_measured_js,
                                         "gripper_measured_js");
        m_interface->AddCommandReadState(*m_state_table, m_setpoint_cp,
                                         "setpoint_cp");

        m_interface->AddCommandWrite(&mtsForceDimensionDevice::servo_cp,
                                     this, "servo_cp");
        m_interface->AddCommandWrite(&mtsForceDimensionDevice::servo_cf,
                                     this, "servo_cf");
        m_interface->AddCommandWrite(&mtsForceDimensionDevice::gripper_servo_jf,
                                     this, "gripper_servo_jf");
        m_interface->AddCommandWrite(&mtsForceDimensionDevice::move_cp,
                                     this, "move_cp");
        m_interface->AddCommandWrite(&mtsForceDimensionDevice::SetGravityCompensation,
                                     this, "SetGravityCompensation");
        m_interface->AddCommandWrite(&mtsForceDimensionDevice::LockOrientation,
                                     this, "LockOrientation");
        m_interface->AddCommandVoid(&mtsForceDimensionDevice::UnlockOrientation,
                                    this, "UnlockOrientation");
        m_interface->AddCommandVoid(&mtsForceDimensionDevice::Freeze,
                                    this, "Freeze");

        // robot State
        m_interface->AddCommandWrite(&mtsForceDimensionDevice::state_command,
                                     this, "state_command", std::string(""));
        m_interface->AddCommandReadState(*m_state_table, m_operating_state,
                                         "device_state");
        m_interface->AddEventWrite(m_operating_state_event, "operating_state",
                                   prmOperatingState());
        // stats
        m_interface->AddCommandReadState(*m_state_table, m_state_table->PeriodStats,
                                         "GetPeriodStatistics");
    }

    // buttons
    const ButtonInterfaces::const_iterator endButtons = buttonInterfaces.end();
    ButtonInterfaces::const_iterator buttonInterface;
    for (buttonInterface = buttonInterfaces.begin();
         buttonInterface != endButtons;
         ++buttonInterface) {
        ButtonData * data = new ButtonData;
        mButtonCallbacks.push_back(data);
        data->Name = (*buttonInterface)->GetName();
        data->Pressed = false;
        (*buttonInterface)->AddEventWrite(data->Function, "Button", prmEventButton());
    }
}


void mtsForceDimensionDevice::Startup(void)
{
    if (!drdIsInitialized(m_device_id)) {
        if (drdAutoInit(m_device_id) < 0) {
            m_interface->SendError(m_name + ": failed to auto init, last reported error is: "
                                   + dhdErrorGetLastStr() + " [id:" + m_device_id_string + "]");
        } else {
            m_interface->SendStatus(m_name + ": properly initialized");
        }
    }

    // set gripper direction
    if (dhdIsLeftHanded(m_device_id)) {
        m_gripper_direction = -1.0;
        m_interface->SendStatus(m_name + ": is left handed");
    } else {
        m_interface->SendStatus(m_name + ": is right handed or symmetrical");
    }

    drdStop(true, m_device_id);

    // update current state
    m_state_table->Start();
    GetRobotData();
    SetControlMode(mtsForceDimension::SERVO_CP);
    m_state_table->Advance();
}

void mtsForceDimensionDevice::Run(void)
{
    m_state_table->Start();

    // process mts commands
    m_interface->ProcessMailBoxes();

    // get robot data
    GetRobotData();

    // control mode
    switch (mControlMode) {
    case mtsForceDimension::SERVO_CF:
        dhdSetForceAndTorqueAndGripperForce(m_servo_cf.Force()[0],
                                            m_servo_cf.Force()[1],
                                            m_servo_cf.Force()[2],
                                            m_servo_cf.Force()[3],
                                            m_servo_cf.Force()[4],
                                            m_servo_cf.Force()[5],
                                            m_gripper_direction * m_gripper_servo_jf,
                                            m_device_id);
        m_setpoint_cp.Position().Assign(m_measured_cp.Position());
        break;
    case mtsForceDimension::SERVO_CP:
        if (m_new_servo_cp) {
            drdTrackPos(m_servo_cp.Goal().Translation().X(),
                        m_servo_cp.Goal().Translation().Y(),
                        m_servo_cp.Goal().Translation().Z(),
                        m_device_id);
            m_new_servo_cp = false;
        }
        break;
    case mtsForceDimension::MOVE_CP:
        // check if the arm is still moving
        if (m_operating_state.IsBusy()
            && (!drdIsMoving(m_device_id))) {
            m_operating_state.IsBusy() = false;
            m_operating_state_event(m_operating_state);
        }
        break;
    default:
        break;
    }
    m_state_table->Advance();
}

void mtsForceDimensionDevice::Cleanup(void)
{
    dhdClose(m_device_id);
}

void mtsForceDimensionDevice::GetButtonNames(std::list<std::string> & result) const
{
    result.clear();
    const ButtonsData::const_iterator end = mButtonCallbacks.end();
    ButtonsData::const_iterator button;
    for (button = mButtonCallbacks.begin();
         button != end;
         ++button) {
        result.push_back((*button)->Name);
    }
}

void mtsForceDimensionDevice::GetRobotData(void)
{
    double rotation[3][3];
    // position
    dhdGetPositionAndOrientationFrame(&m_measured_cp.Position().Translation().X(),
                                      &m_measured_cp.Position().Translation().Y(),
                                      &m_measured_cp.Position().Translation().Z(),
                                      rotation,
                                      m_device_id);
    mRawOrientation.Row(0).Assign(rotation[0]);
    mRawOrientation.Row(1).Assign(rotation[1]);
    mRawOrientation.Row(2).Assign(rotation[2]);
    // apply rotation offset
    m_measured_cp.Position().Rotation() = mRawOrientation * m_rotation_offset;
    m_measured_cp.Valid() = true;

    // velocity
    dhdGetLinearVelocity(&m_measured_cv.VelocityLinear().X(),
                         &m_measured_cv.VelocityLinear().Y(),
                         &m_measured_cv.VelocityLinear().Z(),
                         m_device_id);
    dhdGetAngularVelocityRad(&m_measured_cv.VelocityAngular().X(),
                             &m_measured_cv.VelocityAngular().Y(),
                             &m_measured_cv.VelocityAngular().Z(),
                             m_device_id);
    m_measured_cv.SetValid(true);

    // force
    dhdGetForceAndTorqueAndGripperForce(&m_measured_cf.Force()[0],
                                        &m_measured_cf.Force()[1],
                                        &m_measured_cf.Force()[2],
                                        &m_measured_cf.Force()[3],
                                        &m_measured_cf.Force()[4],
                                        &m_measured_cf.Force()[5],
                                        &m_gripper_measured_js.Effort().at(0),
                                        m_device_id);
    m_measured_cf.SetValid(true);

    // gripper
    dhdGetGripperAngleRad(&m_gripper_measured_js.Position().at(0), m_device_id);
    m_gripper_measured_js.Position().at(0) *= m_gripper_direction;
    dhdGetGripperAngularVelocityRad(&m_gripper_measured_js.Velocity().at(0), m_device_id);

    // buttons
    uint currentButtonMask = dhdGetButtonMask(m_device_id);
    // if any button pressed
    if (mPreviousButtonMask != currentButtonMask) {
        mPreviousButtonMask = currentButtonMask;
        uint index = 0;
        const ButtonsData::iterator buttonsEnd = mButtonCallbacks.end();
        ButtonsData::iterator button;
        for (button = mButtonCallbacks.begin();
             button != buttonsEnd;
             ++button) {
            const bool current =  (currentButtonMask & (1 << index));
            index++;
            if (current != (*button)->Pressed) {
                (*button)->Pressed = current;
                // generate event
                prmEventButton event;
                if (current) {
                    event.SetType(prmEventButton::PRESSED);
                } else {
                    event.SetType(prmEventButton::RELEASED);
                }
                (*button)->Function(event);
            }
        }
    }
}

void mtsForceDimensionDevice::state_command(const std::string & command) {
    if (command == "ENABLE") {
        m_operating_state.State() = prmOperatingState::ENABLED;
    } else if (command == "DISABLE") {
        m_operating_state.State() = prmOperatingState::DISABLED;
    } else {
        m_interface->SendStatus(this->m_name + ": state command \""
                                + command + "\" is not supported yet");
    }
    // always emit event with current device state
    m_interface->SendStatus(this->m_name
                            + ": current state is \""
                            + prmOperatingState::EnumToString(m_operating_state.State()) + "\"");
    m_operating_state_event(m_operating_state);
}

void mtsForceDimensionDevice::SetControlMode(const mtsForceDimension::ControlModeType & mode)
{
    // return if we are already in this mode
    if (mode == mControlMode) {
        return;
    }
    // transition to new mode
    switch (mode) {
    case mtsForceDimension::SERVO_CP:
    case mtsForceDimension::MOVE_CP:
        m_new_servo_cp = false;
        drdRegulatePos(true, m_device_id);
        drdRegulateRot(false, m_device_id);
        drdRegulateGrip(false, m_device_id);
        if (drdStart(m_device_id) < 0) {
            m_interface->SendError(m_name + ": failed to start control look, "
                                   + dhdErrorGetLastStr() + " [id:" + m_device_id_string + "]");
        }
        // start from current position
        m_servo_cp.Goal().Assign(m_measured_cp.Position());
        m_new_servo_cp = true;
        break;
    case mtsForceDimension::SERVO_CF:
        drdRegulatePos(false, m_device_id);
        drdStop(true, m_device_id);
        // start with 0 forces
        m_servo_cf.Force().SetAll(0.0);
        m_gripper_servo_jf = 0.0;
        break;
    default:
        break;
    }
    // assign mode
    mControlMode = mode;
}

void mtsForceDimensionDevice::servo_cf(const prmForceCartesianSet & wrench)
{
    SetControlMode(mtsForceDimension::SERVO_CF);
    m_servo_cf = wrench;
}

void mtsForceDimensionDevice::gripper_servo_jf(const prmForceTorqueJointSet & effortGripper)
{
    if (effortGripper.ForceTorque().size() != 1) {
        m_interface->SendError(m_name + ": effort vector size for gripper servo_jf must be 1 [id:" + m_device_id_string + "]");
        return;
    }
    SetControlMode(mtsForceDimension::SERVO_CF);
    m_gripper_servo_jf = effortGripper.ForceTorque().at(0) * m_gripper_direction;
}

void mtsForceDimensionDevice::servo_cp(const prmPositionCartesianSet & position)
{
    SetControlMode(mtsForceDimension::SERVO_CP);
    m_servo_cp = position;
    m_new_servo_cp = true;
    m_rotation_offset = mRawOrientation.Inverse() * position.Goal().Rotation();
}

void mtsForceDimensionDevice::move_cp(const prmPositionCartesianSet & position)
{
    SetControlMode(mtsForceDimension::MOVE_CP);
    m_servo_cp = position;
    drdMoveToPos(m_servo_cp.Goal().Translation().X(),
                 m_servo_cp.Goal().Translation().Y(),
                 m_servo_cp.Goal().Translation().Z(),
                 false,
                 m_device_id);
    m_rotation_offset = mRawOrientation.Inverse() * position.Goal().Rotation();
    m_operating_state.IsBusy() = true;
    m_operating_state_event(m_operating_state);
}

void mtsForceDimensionDevice::UnlockOrientation(void)
{
    // m_rotation_offset = vctMatRot3();
}

void mtsForceDimensionDevice::Freeze(void)
{
    SetControlMode(mtsForceDimension::SERVO_CP);
    m_servo_cp.Goal().Assign(m_measured_cp.Position());
}

void mtsForceDimensionDevice::LockOrientation(const vctMatRot3 & orientation)
{
    // Rc: rotation current
    // Rd: rotation desired
    // Ro: rotation offset to make the rotation looks like desired
    // Rc * Ro = Rd
    // Ro = Rc_transpose * Rd
    m_rotation_offset = mRawOrientation.Inverse() * orientation;
}

void mtsForceDimensionDevice::SetGravityCompensation(const bool & gravity)
{
    if (gravity) {
        dhdSetGravityCompensation(DHD_ON, m_device_id);
    } else {
        dhdSetGravityCompensation(DHD_OFF, m_device_id);
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

    // required to change asynchronous operation mode
    dhdEnableExpertMode();

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
        const int deviceId = drdOpenID(i);
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
        // create list of buttons based on device type
        std::list<mtsInterfaceProvided *> buttonInterfaces;
        if (dhdGetSystemType(deviceId) == DHD_DEVICE_FALCON) {
            buttonInterfaces.push_back(this->AddInterfaceProvided(deviceName + "-Center"));
            buttonInterfaces.push_back(this->AddInterfaceProvided(deviceName + "-Left"));
            buttonInterfaces.push_back(this->AddInterfaceProvided(deviceName + "-Top"));
            buttonInterfaces.push_back(this->AddInterfaceProvided(deviceName + "-Right"));
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
                                        stateTable, interfaceProvided,
                                        buttonInterfaces);
        mDevices.push_back(device);
    }
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


void mtsForceDimension::GetDeviceNames(std::list<std::string> & result) const
{
    result.clear();
    const DevicesType::const_iterator end = mDevices.end();
    DevicesType::const_iterator device;
    for (device = mDevices.begin();
         device != end;
         ++device) {
        result.push_back((*device)->Name());
    }
}

void mtsForceDimension::GetButtonNames(const std::string & deviceName,
                                       std::list<std::string> & result) const
{
    result.clear();
    const DevicesType::const_iterator end = mDevices.end();
    DevicesType::const_iterator device;
    for (device = mDevices.begin();
         device != end;
         ++device) {
        if ((*device)->Name() == deviceName) {
            (*device)->GetButtonNames(result);
            return;
        }
    }
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
