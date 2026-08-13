#pragma once
#include <string>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include "ftSenseHandler.hpp"
#include <KinovaLiralab.hpp>

class ZForceControl
{
    public:
        ZForceControl(KinovaLiralab::Robot* robot);
        void RunControl(double fzRef);
    private:
        KinovaLiralab::Robot* _robot;
        CanDevice* _forceSensor;
        double _wrench[6];

        // PID
        double _kP = 0.005;
        double _kI = 0.02;
        double _kD = 0.0;

        double _integral = 0.0;
        double _previousError = 0.0;

        // PID output limits
        double _dzMax = 0.02;   // [m] oppure [m/s], a seconda del comando
        double _dzMin = -0.02;

        // Anti-windup
        double _integralMin = -0.05;
        double _integralMax = 0.05;

        std::chrono::steady_clock::time_point _lastTime;
        int frame_to_save = 200;
        std::ofstream* _csvFile;
};

ZForceControl::ZForceControl(KinovaLiralab::Robot* robot) : _robot{robot}
{
    _forceSensor = new CanDevice();

    if(!_forceSensor->Open("can0")) {"Cannot open CAN";return;}

    _forceSensor->InitCompensation();
    _lastTime = std::chrono::steady_clock::now();

    while (!_forceSensor->IsWrenchReady())
    {
        _forceSensor->Receive();
        std::cout << "Waiting for can msgs...\n";
    }

    _forceSensor->SetTare();
    std::cout << "Opening file" << std::endl;
    _csvFile = new std::ofstream("sensor_data.csv");
    *_csvFile << "timestamp,force_z,error\n";
    std::cout << "File opened" << std::endl;
}

void ZForceControl::RunControl(double fzRef)
{
    while (!_forceSensor->IsWrenchReady())
    {
        _forceSensor->Receive();
    }

    KinovaLiralab::RobotState newState = _robot->GetRobotState();
    _forceSensor->ReceiveAllForceAndTorque();
    _forceSensor->GetWrenchCompensated(_wrench, newState);

    // --------------------------------------------------
    // ERROR
    // --------------------------------------------------

    double error = fzRef - _wrench[2];

    // --------------------------------------------------
    // DT
    // --------------------------------------------------

    auto now = std::chrono::steady_clock::now();

    double dt = std::chrono::duration<double>(now - _lastTime).count();
    _lastTime = now;

    // Protezione da dt non valido
    if (dt <= 0.0)
        dt = 1e-3;

    // --------------------------------------------------
    // PROPORTIONAL
    // --------------------------------------------------

    double P = _kP * error;

    // --------------------------------------------------
    // INTEGRAL
    // --------------------------------------------------

    double integralCandidate = _integral + error * dt;

    // --------------------------------------------------
    // DERIVATIVE
    // --------------------------------------------------

    double derivative = (error - _previousError) / dt;
    double D = _kD * derivative;

    // --------------------------------------------------
    // PID NON SATURATO
    // --------------------------------------------------

    double dzUnsaturated =
        P +
        _kI * integralCandidate +
        D;

    // --------------------------------------------------
    // SATURATION
    // --------------------------------------------------

    double dz = std::clamp(
        dzUnsaturated,
        _dzMin,
        _dzMax
    );


    // --------------------------------------------------
    // ANTI-WINDUP
    // --------------------------------------------------

    bool saturatedHigh = dzUnsaturated > _dzMax;
    bool saturatedLow  = dzUnsaturated < _dzMin;

    bool errorWouldIncreaseSaturation =
        (saturatedHigh && error > 0.0) ||
        (saturatedLow  && error < 0.0);

    if (!errorWouldIncreaseSaturation)
    {
        _integral = integralCandidate;

        _integral = std::clamp(
            _integral,
            _integralMin,
            _integralMax
        );
    }


    _previousError = error;

    KDL::Frame T_ee_target = KDL::Frame::Identity();
    T_ee_target.p[2] = -dz;

    KDL::Frame T_w_ee(
        KDL::Rotation(
            newState._eePose[3], newState._eePose[4], newState._eePose[5],
            newState._eePose[6], newState._eePose[7], newState._eePose[8],
            newState._eePose[9], newState._eePose[10], newState._eePose[11]
        ),
        KDL::Vector(newState._eePose[0], newState._eePose[1], newState._eePose[2])
    );

    KDL::Frame T_w_target = T_w_ee * T_ee_target;


    std::cout << T_w_ee.p[0] << ", " << T_w_ee.p[1] << ", " << T_w_ee.p[2] << std::endl;
    std::cout << T_w_target.p[0] << ", " << T_w_target.p[1] << ", " << T_w_target.p[2] << std::endl;
    std::cout << _wrench[2] << std::endl;
    std::cout << " ----- " << std::endl;


    // newState -> eePose -> T_w_ee
    // T_ee_target = identita + dz
    // T_w_target = T_w_ee * T_ee_target
    _robot->SetEquilibriumPose(T_w_target);
    *_csvFile << std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count() << ","
    << _wrench[2] << "," << dz <<"\n";
    frame_to_save--;
    std::cout << frame_to_save << "\n" << std::endl;
    if(frame_to_save == 0)
    {
        _csvFile->close();
    }
}