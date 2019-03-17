/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2014-07-21

  (C) Copyright 2014-2019 Johns Hopkins University (JHU), All Rights Reserved.

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

    // setup CISST interface
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("Device");
    if (interfaceRequired) {
        QMMessage->SetInterfaceRequired(interfaceRequired);
        interfaceRequired->AddFunction("measured_cp", Device.measured_cp);
        interfaceRequired->AddFunction("measured_cf", Device.measured_cf);
        interfaceRequired->AddFunction("gripper_measured_js", Device.gripper_measured_js);
        interfaceRequired->AddFunction("servo_cf", Device.servo_cf);
        interfaceRequired->AddEventHandlerWrite(&mtsForceDimensionQtWidget::OperatingStateEventHandler,
                                                this, "operating_state");
        interfaceRequired->AddFunction("GetPeriodStatistics", Device.GetPeriodStatistics);
        interfaceRequired->AddFunction("Freeze", Device.Freeze);
        interfaceRequired->AddFunction("SetGravityCompensation", Device.SetGravityCompensation);
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

void mtsForceDimensionQtWidget::setupUi(void)
{
    QHBoxLayout * mainLayout = new QHBoxLayout;

    // side by side for 3D position, gripper...
    QVBoxLayout * controlLayout = new QVBoxLayout;
    mainLayout->addLayout(controlLayout);

    // 3D position
    QPCGWidget = new prmPositionCartesianGetQtWidget();
    controlLayout->addWidget(QPCGWidget);

    // wrench
    QFTWidget = new vctForceTorqueQtWidget();
    controlLayout->addWidget(QFTWidget);

    // vectors of values
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

    // system
    QVBoxLayout * systemLayout = new QVBoxLayout();
    mainLayout->addLayout(systemLayout);

    // timing
    QMIntervalStatistics = new mtsIntervalStatisticsQtWidget();
    systemLayout->addWidget(QMIntervalStatistics);

    // messages
    QMMessage->setupUi();
    systemLayout->addWidget(QMMessage);

    // operating state
    QPOState = new prmOperatingStateQtWidget();
    systemLayout->addWidget(QPOState);
    connect(this, SIGNAL(SignalOperatingState(prmOperatingState)),
            this, SLOT(SlotOperatingStateEventHandler(prmOperatingState)));

    systemLayout->addStretch();

    setLayout(mainLayout);
    setWindowTitle("sawForceDimensionSDK");
    resize(sizeHint());
}

void mtsForceDimensionQtWidget::timerEvent(QTimerEvent * CMN_UNUSED(event))
{
    // make sure we should update the display
    if (this->isHidden()) {
        return;
    }

    mtsExecutionResult executionResult;
    executionResult = Device.measured_cp(m_measured_cp);
    if (executionResult) {
        QPCGWidget->SetValue(m_measured_cp);
    }

    executionResult = Device.measured_cf(m_measured_cf);
    if (executionResult) {
        QFTWidget->SetValue(m_measured_cf.F(), m_measured_cf.T(),
                            m_measured_cf.Timestamp());
    }

    executionResult = Device.gripper_measured_js(m_gripper_measured_js);
    if (executionResult) {
        QLPositionGripper->setNum(m_gripper_measured_js.Position().at(0) * cmn180_PI);
    }

    Device.GetPeriodStatistics(IntervalStatistics);
    QMIntervalStatistics->SetValue(IntervalStatistics);
}

void mtsForceDimensionQtWidget::SlotOperatingStateEventHandler(const prmOperatingState & state)
{
    QPOState->SetValue(state);
}

void mtsForceDimensionQtWidget::SlotFreeze(void)
{
    Device.Freeze();
}

void mtsForceDimensionQtWidget::SlotGravityCompensation(void)
{
    Device.SetGravityCompensation(true);
    prmForceCartesianSet wrench;
    Device.servo_cf(wrench);
}

void mtsForceDimensionQtWidget::OperatingStateEventHandler(const prmOperatingState & state)
{
    emit SignalOperatingState(state);
}
