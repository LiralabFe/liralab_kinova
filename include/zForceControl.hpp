#pragma once
#include <string>
#include <stdio.h>
#include "ftSenseHandler.hpp"
#include <KinovaLiralab.hpp>

class ZforceControl
{
    public:
        ZforceControl(KinovaLiralab::Robot* robot);
        RunControl();
    private:
        KinovaLiralab::Robot* _robot;
        CanDevice _forceSensor;
        double _wrench[6];
}

ZforceControl::ZforceControl(KinovaLiralab::Robot* robot) : _robot{robot}
{
    _forceSensor = new CanDevice();

    if(!_forceSensor->Open("can0")) {"Cannot open CAN";return;}

    Eigen::Vector3d com_sensor_payload{0.0, 0.0, 0.08};
    Eigen::Isometry3d T_ee_sensor = Eigen::Isometry3d::Identity();
    Eigen::Vector3d translation(0.0, 0.0, 0.145);
    T_ee_sensor = T_ee_sensor.translate(translation);
    float payloadMass = 0.320 + 0.092;

    _forceSensor->InitCompensation(com_sensor_payload, T_ee_sensor, payloadMass);
}

void ZForceControl::RunControl()
{
    while (!_forceSensor->IsWrenchReady())
    {
        forceSensor->Receive();
        std::cout << "Waiting for can msgs...\n";
    }

    _forceSensor->SetTare();

    KinovaLiralab::RobotState newState = _robot->GetRobotState();
    _forceSensor->ReceiveAllForceAndTorque();
    _forceSensor->GetWrenchCompensated(_wrench, newState);

    // e = Diff force set point and force read
    // e -> PID -> dz
    // newState -> eePose -> T_w_ee
    // T_ee_target = identita + dz
    // T_w_target = T_w_ee * T_ee_target
    // _robot->SetEquilibriumPose(T_w_target)

}