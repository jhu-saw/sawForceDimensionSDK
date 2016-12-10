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

#include <sawForceDimensionSDK/mtsForceDimensionSDK.h>

#include <dhdc.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsForceDimensionSDK, mtsTaskContinuous, mtsTaskContinuousConstructorArg);

void mtsForceDimensionSDK::Init(void)
{
    Internals = new mtsForceDimensionSDKInternals();

    StateTable.AddData(NumberOfThreeDFiducials, "NumberOfThreeDFiducials");
    StateTable.AddData(ThreeDFiducialPosition, "ThreeDFiducialPosition");

    mtsInterfaceProvided * provided = AddInterfaceProvided("Controller");
    if (provided) {
        provided->AddCommandReadState(StateTable, NumberOfThreeDFiducials, "GetNumberOfThreeDFiducials");
        provided->AddCommandReadState(StateTable, ThreeDFiducialPosition, "GetThreeDFiducialPosition");
        provided->AddCommandReadState(StateTable, StateTable.PeriodStats, "GetPeriodStatistics");
    }
}


void mtsForceDimensionSDK::Configure(const std::string & filename)
{

    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: using " << filename << std::endl;

    // initialize fusion track library
    Internals->Library = ftkInit();
    if (!Internals->Library) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: unable to initialize ("
                                 << this->GetName() << ")" << std::endl;
        return;
    }

    for (size_t i = 0; i < 10; i++) {
        // search for devices
        ftkError error = ftkEnumerateDevices(Internals->Library,
                                             mtsForceDimensionSDKDeviceEnum,
                                             &(Internals->Device));
        if (error != FTK_OK) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: unable to enumerate devices ("
                                     << this->GetName() << ")" << std::endl;
            ftkClose(Internals->Library);
        }

        if (Internals->Device != 0LL) {
            break;
        }
    }
    if (Internals->Device == 0LL) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: no device connected ("
                                 << this->GetName() << ")" << std::endl;
        ftkClose(Internals->Library);
        return;
    }

    if (filename == "") {
        return;
    }

    // read JSON file passed as param, see configForceDimensionSDK.json for an example
    std::ifstream jsonStream;
    jsonStream.open(filename.c_str());

    Json::Value jsonConfig, jsonValue;
    Json::Reader jsonReader;
    if (!jsonReader.parse(jsonStream, jsonConfig)) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to parse configuration" << std::endl
                                 << jsonReader.getFormattedErrorMessages();
        Internals->Configured = false;
        return;
    }

    // allows the use of relative paths for ini files
    cmnPath configPath(cmnPath::GetWorkingDirectory());


    const Json::Value jsonTools = jsonConfig["tools"];
    for (unsigned int index = 0; index < jsonTools.size(); ++index) {
        jsonValue = jsonTools[index];

        std::string toolName = jsonValue["name"].asString();

        // make sure toolName is valid
        if (toolName == "") {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: invalid tool name found in " << filename << std::endl;
        } else {
            std::string iniFile = jsonValue["ini-file"].asString();
            std::string fullname = configPath.Find(iniFile);
            // make sure ini file is valid
            if (cmnPath::Exists(fullname)) {
                CMN_LOG_CLASS_INIT_VERBOSE << "Configure: called AddToolIni with toolName: " << toolName << " and ini file location: " << iniFile << std::endl;
                AddToolIni(toolName, fullname);
            } else {
                CMN_LOG_CLASS_INIT_ERROR << "Configure: ini file " << iniFile
                                         << " not found in path (" << configPath << ")" << std::endl;
            }
        }
    }
}


void mtsForceDimensionSDK::Startup(void)
{
    CMN_LOG_CLASS_RUN_ERROR << "Startup" << std::endl;
}


void mtsForceDimensionSDK::Run(void)
{
    // process mts commands
    ProcessQueuedCommands();

    // get latest frame from fusion track library/device
    if (ftkGetLastFrame(Internals->Library,
                        Internals->Device,
                        Internals->Frame,
                        100 ) != FTK_OK) {
        CMN_LOG_CLASS_RUN_DEBUG << "Run: timeout on ftkGetLastFrame" << std::endl;
        return;
    }

    // check results of last frame
    switch (Internals->Frame->markersStat) {
    case QS_WAR_SKIPPED:
        CMN_LOG_CLASS_RUN_ERROR << "Run: marker fields in the frame are not set correctly" << std::endl;
    case QS_ERR_INVALID_RESERVED_SIZE:
        CMN_LOG_CLASS_RUN_ERROR << "Run: frame.markersVersionSize is invalid" << std::endl;
    default:
        CMN_LOG_CLASS_RUN_ERROR << "Run: invalid status" << std::endl;
    case QS_OK:
        break;
    }

    // make sure we're not getting more markers than allocated
    size_t count = Internals->Frame->markersCount;
    if (count > Internals->NumberOfMarkers) {
        CMN_LOG_CLASS_RUN_WARNING << "Run: marker overflow, please increase number of markers.  Only the first "
                                  << Internals->NumberOfMarkers << " marker(s) will processed." << std::endl;
        count = Internals->NumberOfMarkers;
    }

    // initialize all tools
    const ToolsType::iterator end = Tools.end();
    ToolsType::iterator iter;
    for (iter = Tools.begin(); iter != end; ++iter) {
        iter->second->StateTable.Start();
        iter->second->Position.SetValid(false);
    }

    // for each marker, get the data and populate corresponding tool
    for (size_t index = 0; index < count; ++index) {
        ftkMarker * currentMarker = &(Internals->Markers[index]);
        // find the appropriate tool
        if (Internals->GeometryIdToTool.find(currentMarker->geometryId) == Internals->GeometryIdToTool.end()) {
            CMN_LOG_CLASS_RUN_WARNING << "Run: found a geometry Id not registered using AddTool, this marker will be ignored ("
                                      << this->GetName() << ")" << std::endl;
        }
        else {
            mtsForceDimensionSDKTool * tool = Internals->GeometryIdToTool.at(currentMarker->geometryId);
            tool->Position.SetValid(true);
            tool->Position.Position().Translation().Assign(currentMarker->translationMM[0],
                                                           currentMarker->translationMM[1],
                                                           currentMarker->translationMM[2]);
            for (size_t row = 0; row < 3; ++row) {
                for (size_t col = 0; col < 3; ++col) {
                    tool->Position.Position().Rotation().Element(row, col) = currentMarker->rotation[row][col];
                }
            }

            tool->RegistrationError = currentMarker->registrationErrorMM;

            /*
              printf("Marker:\n");
              printf("XYZ (%.2f %.2f %.2f)\n",
              currentMarker->translationMM[0],
              currentMarker->translationMM[1],
              currentMarker->translationMM[2]);
            */
        }
    }

    // finalize all tools
    for (iter = Tools.begin(); iter != end; ++iter) {
        iter->second->StateTable.Advance();
    }

    // ---- 3D Fiducials ---
    switch (Internals->Frame->threeDFiducialsStat) {
    case QS_WAR_SKIPPED:
        CMN_LOG_CLASS_RUN_ERROR << "Run: 3D status fields in the frame is not set correctly" << std::endl;
    case QS_ERR_INVALID_RESERVED_SIZE:
        CMN_LOG_CLASS_RUN_ERROR << "Run: frame.threeDFiducialsVersionSize is invalid" << std::endl;
    default:
        CMN_LOG_CLASS_RUN_ERROR << "Run: invalid status" << std::endl;
    case QS_OK:
        break;
    }

    ThreeDFiducialPosition.clear();
    NumberOfThreeDFiducials = Internals->Frame->threeDFiducialsCount;
    ThreeDFiducialPosition.resize(NumberOfThreeDFiducials);

    //printf("3D fiducials:\n");
    for (uint32 m = 0; m < NumberOfThreeDFiducials; m++) {
        /*
          printf("\tINDEXES (%u %u)\t XYZ (%.2f %.2f %.2f)\n\t\tEPI_ERR: %.2f\tTRI_ERR: %.2f\tPROB: %.2f\n",
          Internals->threedFiducials[m].leftIndex,
          Internals->threedFiducials[m].rightIndex,
          Internals->threedFiducials[m].positionMM.x,
          Internals->threedFiducials[m].positionMM.y,
          Internals->threedFiducials[m].positionMM.z,
          Internals->threedFiducials[m].epipolarErrorPixels,
          Internals->threedFiducials[m].triangulationErrorMM,
          Internals->threedFiducials[m].probability);
        */
        ThreeDFiducialPosition[m].X() = Internals->threedFiducials[m].positionMM.x;
        ThreeDFiducialPosition[m].Y() = Internals->threedFiducials[m].positionMM.y;
        ThreeDFiducialPosition[m].Z() = Internals->threedFiducials[m].positionMM.z;
    }
}

void mtsForceDimensionSDK::Cleanup(void)
{
    ftkClose(Internals->Library);
}


bool mtsForceDimensionSDK::AddToolIni(const std::string & toolName, const std::string & fileName)
{

    // check if this tool already exists
    mtsForceDimensionSDKTool * tool = Tools.GetItem(toolName);
    if (tool) {
        CMN_LOG_CLASS_INIT_ERROR << "AddToolIni: " << tool->Name << " already exists" << std::endl;
        return false;
    }

    // make sure we can find and load this tool ini file
    ftkError error;
    ftkGeometry geometry;
    switch (loadGeometry(Internals->Library, Internals->Device, fileName, geometry)) {
    case 1:
        CMN_LOG_CLASS_INIT_VERBOSE << "AddToolIni: loaded " << fileName << " from installation directory"
                                   << std::endl;
    case 0:
        error = ftkSetGeometry(Internals->Library, Internals->Device, &geometry);
        if (error != FTK_OK) {
            CMN_LOG_CLASS_INIT_ERROR << "AddToolIni: unable to set geometry for tool "
                                     << fileName << " (" << this->GetName() << ")" << std::endl;
            return false;
        }
        else {
            CMN_LOG_CLASS_INIT_ERROR << "AddToolIni: unable to set geometry for tool "
                                     << fileName << " (" << this->GetName() << ") but received FTK_OK"
                                     << std::endl;
        }
        break;
    default:
        CMN_LOG_CLASS_INIT_ERROR << "AddToolIni: error, cannot load geometry file "
                                 << fileName << std::endl;
        return false;
    }

    // make sure there is no such geometry Id yet
    const mtsForceDimensionSDKInternals::GeometryIdToToolMap::const_iterator
        toolIterator = Internals->GeometryIdToTool.find(geometry.geometryId);
    if (toolIterator != Internals->GeometryIdToTool.end()) {
        CMN_LOG_CLASS_INIT_ERROR << "AddToolIni: error, found an existing tool with the same Id "
                                 << geometry.geometryId << " for " << fileName << std::endl;
        return false;
    }

    // finally create a cisst tool structure
    tool = new mtsForceDimensionSDKTool(toolName);

    // create an interface for tool
    tool->Interface = AddInterfaceProvided(toolName);
    if (!tool->Interface) {
        CMN_LOG_CLASS_INIT_ERROR << "AddToolIni: " << tool->Name << " already exists" << std::endl;
        delete tool;
        return false;
    }

    // register newly created tool
    this->Tools.AddItem(toolName, tool);
    Internals->GeometryIdToTool[geometry.geometryId] = tool;

    // add data for this tool and populate tool interface
    tool->StateTable.SetAutomaticAdvance(false);
    this->AddStateTable(&(tool->StateTable));
    tool->StateTable.AddData(tool->Position, "Position");
    tool->StateTable.AddData(tool->RegistrationError, "RegistrationError");
    tool->Interface->AddCommandReadState(tool->StateTable, tool->Position, "GetPositionCartesian");
    tool->Interface->AddCommandReadState(tool->StateTable, tool->RegistrationError, "GetRegistrationError");
    tool->Interface->AddCommandReadState(tool->StateTable,
                                         tool->StateTable.PeriodStats,
                                         "GetPeriodStatistics");
    return true;
}


std::string mtsForceDimensionSDK::GetToolName(const size_t index) const
{
    ToolsType::const_iterator toolIterator = Tools.begin();
    if (index >= Tools.size()) {
        CMN_LOG_CLASS_RUN_ERROR << "GetToolName: requested index is out of range" << std::endl;
        return "";
    }
    for (unsigned int i = 0; i < index; i++) {
        toolIterator++;
    }
    return toolIterator->first;
}
