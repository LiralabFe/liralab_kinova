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
    void SetTare();

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
};

/*
int main()
{
    constexpr uint32_t COMMAND_ID = 0x20D; // Sostituisci con quello corretto

    CanDevice can;
    double firstWrench[6];
    bool isFirst = true;

    if (!can.Open("can0")){
        std::cerr << "Errore apertura CAN\n";
        return -1;
    }

    can.StartTransmission();
    can.BlockTransmission();
    can.GetFullScale();
    can.SetDataRate(1000000);
    can.StartTransmission();
    
    while (true)
    {
        can.Receive();
        if(can.IsWrenchReady())
        {
            double wrench[6];
            can.GetWrench(wrench);
            if(isFirst)
            {
                isFirst = false;
                firstWrench[0] = wrench[0];
                firstWrench[1] = wrench[1];
                firstWrench[2] = wrench[2];
                firstWrench[3] = wrench[3];
                firstWrench[4] = wrench[4];
                firstWrench[5] = wrench[5];
            }
            for(int i = 0; i < 6; i++)
            {

                if(i < 3){
                    std::cout << "FORCE: " << wrench[i] - firstWrench[i]<< std::endl;
                }
                else{
                    std::cout << "TORQUE: " << wrench[i] << std::endl;
                }
            }
            //std::cout << "____" << std::endl;
        }
    }

    return 0;
}
*/