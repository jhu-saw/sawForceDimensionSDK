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
#include <QPushButton>

// cisst
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <sawForceDimensionSDK/mtsForceDimensionQtWidget.h>
#include <cisstParameterTypes/prmForceCartesianSet.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsForceDimensionQtWidget, mtsComponent, std::string);

mtsForceDimensionQtWidget::mtsForceDimensionQtWidget(const std::string & componentName, double periodInSeconds):
    mtsComponent(componentName),
    TimerPeriodInMilliseconds(periodInSeconds)
{
    QMMessage = new mtsMessageQtWidget();

    // Setup CISST Interface
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("Device");
    if (interfaceRequired) {
        QMMessage->SetInterfaceRequired(interfaceRequired);
        interfaceRequired->AddFunction("GetPositionCartesian", Device.GetPositionCartesian);
        interfaceRequired->AddFunction("GetWrenchBody", Device.GetWrenchBody);
        interfaceRequired->AddFunction("GetStateGripper", Device.GetStateGripper);
        interfaceRequired->AddFunction("GetPeriodStatistics", Device.GetPeriodStatistics);
        interfaceRequired->AddFunction("Freeze", Device.Freeze);
        interfaceRequired->AddFunction("SetGravityCompensation", Device.SetGravityCompensation);
        interfaceRequired->AddFunction("SetWrenchBody", Device.SetWrenchBody);
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
    if (executionResult) {
        QPCGWidget->SetValue(PositionCartesian);
    }

    executionResult = Device.GetWrenchBody(Wrench);
    if (executionResult) {
        QFTWidget->SetValue(Wrench.F(), Wrench.T(), Wrench.Timestamp());
    }

    executionResult = Device.GetStateGripper(StateGripper);
    if (executionResult) {
        QLPositionGripper->setNum(StateGripper.Position().at(0) * cmn180_PI);
    }

    Device.GetPeriodStatistics(IntervalStatistics);
    QMIntervalStatistics->SetValue(IntervalStatistics);
}

void mtsForceDimensionQtWidget::setupUi(void)
{
    QHBoxLayout * mainLayout = new QHBoxLayout;

    // Side by side for 3D position, gripper...
    QVBoxLayout * controlLayout = new QVBoxLayout;
    mainLayout->addLayout(controlLayout);

    // 3D position
    QPCGWidget = new prmPositionCartesianGetQtWidget();
    controlLayout->addWidget(QPCGWidget);

    // Wrench
    QFTWidget = new vctForceTorqueQtWidget();
    controlLayout->addWidget(QFTWidget);

    // Vectors of values
    QGridLayout * gridLayout = new QGridLayout;
    controlLayout->addLayout(gridLayout);

    gridLayout->setSpacing(1);
    int row = 0;
    gridLayout->addWidget(new QLabel("Gripper"), row, 0);
    QLPositionGripper = new QLabel();
    gridLayout->addWidget(QLPositionGripper, row, 1);
    row++;

    QPushButton * freezeButton = new QPushButton("Freeze");
    controlLayout->addWidget(freezeButton);
    connect(freezeButton, SIGNAL(clicked()),
            this, SLOT(SlotFreeze()));

    QPushButton * gravityCompensationButton = new QPushButton("Gravity compensation");
    controlLayout->addWidget(gravityCompensationButton);
    connect(gravityCompensationButton, SIGNAL(clicked()),
            this, SLOT(SlotGravityCompensation()));

    controlLayout->addStretch();

    // System
    QVBoxLayout * systemLayout = new QVBoxLayout();
    mainLayout->addLayout(systemLayout);

    // Timing
    QMIntervalStatistics = new mtsIntervalStatisticsQtWidget();
    systemLayout->addWidget(QMIntervalStatistics);

    // Messages
    QMMessage->setupUi();
    systemLayout->addWidget(QMMessage);

    systemLayout->addStretch();

    setLayout(mainLayout);
    setWindowTitle("sawForceDimensionSDK");
    resize(sizeHint());
}

void mtsForceDimensionQtWidget::SlotFreeze(void)
{
    Device.Freeze();
}

void mtsForceDimensionQtWidget::SlotGravityCompensation(void)
{
    Device.SetGravityCompensation(true);
    prmForceCartesianSet wrench;
    Device.SetWrenchBody(wrench);
}
