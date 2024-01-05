/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2014-07-21

  (C) Copyright 2014-2023 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <QLabel>
#include <QtGui>
#include <QMessageBox>
#include <QPushButton>

// cisst
#include <cisstVector/vctForceTorqueQtWidget.h>
#include <cisstMultiTask/mtsManagerLocal.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsIntervalStatisticsQtWidget.h>
#include <cisstMultiTask/mtsMessageQtWidget.h>
#include <cisstParameterTypes/prmForceCartesianSet.h>
#include <cisstParameterTypes/prmEventButtonQtWidget.h>
#include <cisstParameterTypes/prmPositionCartesianGetQtWidget.h>
#include <cisstParameterTypes/prmStateJointQtWidget.h>
#include <cisstParameterTypes/prmOperatingStateQtWidget.h>

#include <sawForceDimensionSDK/mtsForceDimensionQtWidget.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsForceDimensionQtWidget, mtsComponent, std::string);

mtsForceDimensionQtWidget::mtsForceDimensionQtWidget(const std::string & componentName, double periodInSeconds):
    mtsComponent(componentName),
    TimerPeriodInMilliseconds(periodInSeconds)
{
    QMMessage = new mtsMessageQtWidget();
    QPOState = new prmOperatingStateQtWidget();

    // setup interface
    m_device_interface = AddInterfaceRequired("Device");
    if (m_device_interface) {
        QMMessage->SetInterfaceRequired(m_device_interface);
        QPOState->SetInterfaceRequired(m_device_interface);
        m_device_interface->AddFunction("measured_cp", Device.measured_cp);
        m_device_interface->AddFunction("body/measured_cf", Device.body_measured_cf);
        m_device_interface->AddFunction("gripper/measured_js", Device.gripper_measured_js);
        m_device_interface->AddFunction("hold", Device.hold);
        m_device_interface->AddFunction("free", Device.free);
        m_device_interface->AddFunction("period_statistics", Device.period_statistics);
        m_device_interface->AddFunction("get_button_names", Device.get_button_names);
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
    // get button names
    typedef std::list<std::string> BList;
    BList buttons;
    if (Device.get_button_names.IsValid()) {
        if (Device.get_button_names(buttons)) {
            mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
            // name of device we're connected to
            std::string device_name =  m_device_interface->GetConnectedInterface()->GetComponent()->GetName();
            const BList::const_iterator end = buttons.end();
            for (BList::const_iterator iter = buttons.begin();
                 iter != end;
                 ++iter) {
                QPBWidgetComponent->AddEventButton(*iter);
            }
            // connect all the interfaces
            for (BList::const_iterator iter = buttons.begin();
                 iter != end;
                 ++iter) {
                componentManager->Connect(QPBWidgetComponent->GetName(), *iter,
                                          device_name, *iter);
            }
        }
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

    // gripper
    QSJWidget = new prmStateJointQtWidget();
    controlLayout->addWidget(QSJWidget);

    // buttons widget
    QPBWidgetComponent = new prmEventButtonQtWidgetComponent(GetName() + "-buttons");
    QPBWidgetComponent->SetNumberOfColumns(2);
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    componentManager->AddComponent(QPBWidgetComponent);
    controlLayout->addWidget(QPBWidgetComponent);

    QPushButton * holdButton = new QPushButton("Hold");
    controlLayout->addWidget(holdButton);
    connect(holdButton, SIGNAL(clicked()),
            this, SLOT(SlotHold()));

    QPushButton * freeButton = new QPushButton("Free");
    controlLayout->addWidget(freeButton);
    connect(freeButton, SIGNAL(clicked()),
            this, SLOT(SlotFree()));

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
    QPOState->setupUi();
    systemLayout->addWidget(QPOState);

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

    executionResult = Device.body_measured_cf(m_body_measured_cf);
    if (executionResult) {
        QFTWidget->SetValue(m_body_measured_cf.F(), m_body_measured_cf.T(),
                            m_body_measured_cf.Timestamp());
    }

    executionResult = Device.gripper_measured_js(m_gripper_measured_js);
    if (executionResult) {
        QSJWidget->SetValue(m_gripper_measured_js);
    }

    Device.period_statistics(IntervalStatistics);
    QMIntervalStatistics->SetValue(IntervalStatistics);
}

void mtsForceDimensionQtWidget::SlotHold(void)
{
    Device.hold();
}

void mtsForceDimensionQtWidget::SlotFree(void)
{
    Device.free();
}
