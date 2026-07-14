#pragma once
#include <iostream>
#include <cstring>
#include <stdint.h>
#include <stdio.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <iomanip>
#include <unistd.h>
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "RobotState.hpp"

/*

    Bisogna abilitare SocketCAN per vedere CAN0:
    - sudo rmmod pcan
    - sudo modprobe peak_usb
    - sudo ip link set can0 up type can bitrate 1000000
*/

class CanDevice
{
    public: /* PUBLIC METHODS */
    bool Open(const std::string& interfaceName);
    bool Send(uint32_t id, uint8_t b0, uint8_t b1);
    int Receive();
    void BlockTransmission();
    void StartTransmission();
    void SetDataRate(int dataRate);
    void GetFullScale();
    ~CanDevice();
    can_frame GetInFrame();
    bool IsWrenchReady();
    void GetWrench(double* wrench);
    void GetWrenchCompensated(double* wrench, KinovaLiralab::RobotState robotState, bool subtract_gravity=true);
    void SetTare();
    void InitCompensation(Eigen::Vector3d com_sensor_payload, Eigen::Isometry3d T_ee_sensor, double payload_mass);

    private: /* PRIVATE METHODS */
    void GetValues(unsigned short int *value);
    void PrintSuggestions();

    /* PRIVATE VARIABLES */
    bool force_readed = false;
    bool torque_readed = false;
    int fd = -1;
    uint16_t in_force_pre_scale[3];
    uint16_t in_torque_pre_scale[3];
    unsigned short scales_vect[6];
    can_frame in_frame;
    unsigned short in_value;
    double current_tare[6];
    bool set_tare = false;

    const uint32_t COMMAND_ID = 0x20D;

    // For gravity compensation
    bool compensationIsReady = false;
    const Eigen::Vector3d g_world{0.0, 0.0, -9.81};
    double payload_mass;                                // Payload mass after the sensor
    Eigen::Vector3d com_sensor_payload;          // Payload CoM wrt force sensor [m]
    Eigen::Isometry3d T_ee_sensor;               // sensor pose wrt EE
};