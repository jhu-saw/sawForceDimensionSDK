/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2014-07-21

  (C) Copyright 2014-2016 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

// system include
#include <iostream>

// Qt include
#include <QString>
#include <QtGui>

// cisst
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <sawAtracsysFusionTrack/mtsAtracsysFusionTrackToolQtWidget.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsAtracsysFusionTrackToolQtWidget, mtsComponent, std::string);

mtsAtracsysFusionTrackToolQtWidget::mtsAtracsysFusionTrackToolQtWidget(const std::string & componentName, double periodInSeconds):
    mtsComponent(componentName),
    TimerPeriodInMilliseconds(periodInSeconds)
{
    // Setup CISST Interface
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("Tool");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("GetPositionCartesian", Tool.GetPositionCartesian);
        interfaceRequired->AddFunction("GetRegistrationError", Tool.GetRegistrationError);
        interfaceRequired->AddFunction("GetPeriodStatistics", Tool.GetPeriodStatistics);
    }
    setupUi();
    startTimer(TimerPeriodInMilliseconds); // ms
}

void mtsAtracsysFusionTrackToolQtWidget::Configure(const std::string &filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}

void mtsAtracsysFusionTrackToolQtWidget::Startup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "mtsAtracsysFusionTrackToolQtWidget::Startup" << std::endl;
    if (!parent()) {
        show();
    }
}

void mtsAtracsysFusionTrackToolQtWidget::Cleanup(void)
{
    this->hide();
    CMN_LOG_CLASS_INIT_VERBOSE << "mtsAtracsysFusionTrackToolQtWidget::Cleanup" << std::endl;
}

void mtsAtracsysFusionTrackToolQtWidget::closeEvent(QCloseEvent * event)
{
    int answer = QMessageBox::warning(this, tr("mtsAtracsysFusionTrackToolQtWidget"),
                                      tr("Do you really want to quit this application?"),
                                      QMessageBox::No | QMessageBox::Yes);
    if (answer == QMessageBox::Yes) {
        event->accept();
        QCoreApplication::exit();
    } else {
        event->ignore();
    }
}

void mtsAtracsysFusionTrackToolQtWidget::timerEvent(QTimerEvent * CMN_UNUSED(event))
{
    // make sure we should update the display
    if (this->isHidden()) {
        return;
    }

    mtsExecutionResult executionResult;
    executionResult = Tool.GetPositionCartesian(PositionCartesian);
    if (!executionResult) {
        CMN_LOG_CLASS_RUN_ERROR << "Tool.GetPositionCartesian failed, \""
                                << executionResult << "\"" << std::endl;
    }
    else {
        QFRPositionCartesianWidget->SetValue(PositionCartesian.Position());
        QLValid->setNum(PositionCartesian.Valid());
    }

    executionResult = Tool.GetRegistrationError(RegistrationError);
    if (!executionResult) {
        CMN_LOG_CLASS_RUN_ERROR << "Tool.GetRegistrationError failed, \""
                                << executionResult << "\"" << std::endl;
    }
    else {
        QLRegistrationError->setNum(RegistrationError);
    }
    Tool.GetPeriodStatistics(IntervalStatistics);
    QMIntervalStatistics->SetValue(IntervalStatistics);
}

void mtsAtracsysFusionTrackToolQtWidget::setupUi(void)
{
    QVBoxLayout * mainLayout = new QVBoxLayout;

    // Side by side for 3D position and timing
    QHBoxLayout * topLayout = new QHBoxLayout;
    mainLayout->addLayout(topLayout);

    // 3D position
    QFRPositionCartesianWidget = new vctQtWidgetFrameDoubleRead(vctQtWidgetRotationDoubleRead::OPENGL_WIDGET);
    topLayout->addWidget(QFRPositionCartesianWidget);

    // Timing
    QVBoxLayout * timingLayout = new QVBoxLayout();
    QMIntervalStatistics = new mtsQtWidgetIntervalStatistics();
    timingLayout->addWidget(QMIntervalStatistics);
    timingLayout->addStretch();
    topLayout->addLayout(timingLayout);

    // Vectors of values
    QGridLayout * gridLayout = new QGridLayout;
    mainLayout->addLayout(gridLayout);

    gridLayout->setSpacing(1);
    int row = 0;
    gridLayout->addWidget(new QLabel("Valid"), row, 0);
    QLValid = new QLabel();
    gridLayout->addWidget(QLValid, row, 1);
    row++;
    gridLayout->addWidget(new QLabel("Registration Error"), row, 0);
    QLRegistrationError = new QLabel();
    gridLayout->addWidget(QLRegistrationError, row, 1);
    row++;

    setLayout(mainLayout);
    setWindowTitle("Tool");
    resize(sizeHint());
}
