/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2014-07-21

  (C) Copyright 2014-2017 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <QMessageBox>

// cisst
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <sawForceDimensionSDK/mtsForceDimensionQtWidget.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsForceDimensionQtWidget, mtsComponent, std::string);

mtsForceDimensionQtWidget::mtsForceDimensionQtWidget(const std::string & componentName, double periodInSeconds):
    mtsComponent(componentName),
    TimerPeriodInMilliseconds(periodInSeconds)
{
    // Setup CISST Interface
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("Device");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("GetPositionCartesian", Device.GetPositionCartesian);
        interfaceRequired->AddFunction("GetStateGripper", Device.GetStateGripper);
        interfaceRequired->AddFunction("GetPeriodStatistics", Device.GetPeriodStatistics);
    }
    setupUi();
    startTimer(TimerPeriodInMilliseconds); // ms
}

void mtsForceDimensionQtWidget::Configure(const std::string &filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}

void mtsForceDimensionQtWidget::Startup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "mtsForceDimensionQtWidget::Startup" << std::endl;
    if (!parent()) {
        show();
    }
}

void mtsForceDimensionQtWidget::Cleanup(void)
{
    this->hide();
    CMN_LOG_CLASS_INIT_VERBOSE << "mtsForceDimensionQtWidget::Cleanup" << std::endl;
}

void mtsForceDimensionQtWidget::closeEvent(QCloseEvent * event)
{
    int answer = QMessageBox::warning(this, tr("mtsForceDimensionQtWidget"),
                                      tr("Do you really want to quit this application?"),
                                      QMessageBox::No | QMessageBox::Yes);
    if (answer == QMessageBox::Yes) {
        event->accept();
        QCoreApplication::exit();
    } else {
        event->ignore();
    }
}

void mtsForceDimensionQtWidget::timerEvent(QTimerEvent * CMN_UNUSED(event))
{
    // make sure we should update the display
    if (this->isHidden()) {
        return;
    }

    mtsExecutionResult executionResult;
    executionResult = Device.GetPositionCartesian(PositionCartesian);
    if (!executionResult) {
        CMN_LOG_CLASS_RUN_ERROR << ".GetPositionCartesian failed, \""
                                << executionResult << "\"" << std::endl;
    }
    else {
        QFRPositionCartesianWidget->SetValue(PositionCartesian.Position());
    }

    executionResult = Device.GetStateGripper(StateGripper);
    QLPositionGripper->setNum(StateGripper.Position().at(0) * cmn180_PI);

    Device.GetPeriodStatistics(IntervalStatistics);
    QMIntervalStatistics->SetValue(IntervalStatistics);
}

void mtsForceDimensionQtWidget::setupUi(void)
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
    gridLayout->addWidget(new QLabel("Gripper"), row, 0);
    QLPositionGripper = new QLabel();
    gridLayout->addWidget(QLPositionGripper, row, 1);
    row++;

    setLayout(mainLayout);
    setWindowTitle("Device");
    resize(sizeHint());
}
