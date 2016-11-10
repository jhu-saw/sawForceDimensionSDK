/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2014-07-17

  (C) Copyright 2014-2016 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsAtracsysFusionTrack_h
#define _mtsAtracsysFusionTrack_h

#include <cisstCommon/cmnPath.h>
#include <cisstMultiTask/mtsTaskContinuous.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>

#include <json/json.h> // in order to read config file

#include <sawAtracsysFusionTrack/sawAtracsysFusionTrackExport.h>  // always include last

// forward declarations for internal data
class mtsAtracsysFusionTrackInternals;
class mtsAtracsysFusionTrackTool;

/*!
  \todo Create InitLibrary method and call it in Configure or AddTool is not already done
  \todo Add flag to check if ftkInit has been called and then check in AddTool and Startup, report error if not
  \todo Add method AddTool to add tool from geometry as std::vector<vct3> + geometry Id
  \todo Use method AddTool in AddToolIni
  \todo Can this thing beep on command?
*/
class CISST_EXPORT mtsAtracsysFusionTrack: public mtsTaskContinuous
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

 public:
    inline mtsAtracsysFusionTrack(const std::string & componentName):
        mtsTaskContinuous(componentName, 100) {
        Construct();
    }

    inline mtsAtracsysFusionTrack(const mtsTaskContinuousConstructorArg & arg):
        mtsTaskContinuous(arg) {
        Construct();
    }

    ~mtsAtracsysFusionTrack(void) {};

    /*! Configure the device.  If the method is called without a file
      name (or an empty string), this method initializes the hardware.
      Users will have to call the AddToolInit later to add tools.  If
      a file name is provided, the methods assumes it corresponds to a
      JSON file containing an array of "tools", each of them being
      defined by a "name" and an "ini-file".  The path for the ini
      file can be either absolute or relative to the application's
      working directory. */
    void Configure(const std::string & filename = "");

    void Startup(void);

    void Run(void);

    void Cleanup(void);

    bool AddToolIni(const std::string & toolName, const std::string & fileName);

    inline size_t GetNumberOfTools(void) const {
        return Tools.size();
    }

    std::string GetToolName(const size_t index) const;

protected:

    /*! Code called by all constructors. */
    void Construct(void);

    mtsAtracsysFusionTrackInternals * Internals;
    typedef cmnNamedMap<mtsAtracsysFusionTrackTool> ToolsType;
    ToolsType Tools;

    int NumberOfThreeDFiducials;
    std::vector<vct3> ThreeDFiducialPosition;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsAtracsysFusionTrack);

#endif  // _mtsAtracsysFusionTrack_h
