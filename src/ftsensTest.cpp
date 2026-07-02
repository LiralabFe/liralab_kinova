#include <iostream>
#include <cstring>
#include <stdint.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <iomanip>
#include<unistd.h>
#include <math.h>

/*

    Bisogna abilitare SocketCAN per vedere CAN0:
    - sudo rmmod pcan
    - sudo modprobe peak_usb
    - sudo ip link set can0 up type can bitrate 1000000
*/

class CanDevice
{
public:
    bool Open(const std::string& interfaceName)
    {
        fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);

        if (fd < 0)
            return false;

        struct ifreq ifr{};
        std::strcpy(ifr.ifr_name, interfaceName.c_str());

        if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0)
            return false;

        struct sockaddr_can addr{};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0)
            return false;

        return true;
    }

    bool Send(uint32_t id, uint8_t b0, uint8_t b1)
    {
        struct can_frame frame{};
        frame.can_id = id;
        frame.can_dlc = 2;
        frame.data[0] = b0;
        frame.data[1] = b1;

        return write(fd, &frame, sizeof(frame)) == sizeof(frame);
    }

    int Receive()
    {
        bool result = read(fd, &in_frame, sizeof(in_frame)) > 0;

        switch (in_frame.can_id)
        {
        case 0x3DA: //FORCE ID
        {
            std::cout << std::hex << (unsigned short)in_frame.data[1] << std::endl;
            std::cout << std::hex << (unsigned short)in_frame.data[0] << std::endl;
            unsigned short tmp;
            int currForce = 0;
            for (int i = 1; i < (unsigned short )in_frame.can_dlc; i = i + 2) {
                tmp = (((unsigned short)in_frame.data[i]) << 8) & 0xFF00;
                tmp += ((unsigned short )in_frame.data[i-1]) & 0x00FF;
                in_force_pre_scale[currForce] = (uint16_t)tmp;
                currForce++;
            }

            force_readed = true;
            return 0;
        }
        case 0x3DB: // TORQUE ID
        {
            int currTorque = 0;
            int16_t tmp;
            for (int i = 1; i < (unsigned short )in_frame.can_dlc; i = i + 2)
            {
                tmp = (((unsigned short )in_frame.data[i]) << 8) & 0xFF00;
                tmp += ((unsigned short )in_frame.data[i-1]) & 0x00FF;

                in_torque_pre_scale[currTorque] = tmp;
                currTorque++;
            }
            torque_readed = true;
            return 0;
        }
        case 0x2D0: // RESPONSE_ID
            if(in_frame.can_dlc == 0)
                return 1;
            if (in_frame.can_dlc==4)
            {
                GetValues(&in_value);
                return 2;
            }
            break;
        }

        return -1;
    }

    void BlockTransmission()
    {
        Send(0x20D, 0x7, 0x1);

        int returnValue = Receive();
        //read pack untill the ack
        while ((returnValue != 1))
            returnValue = Receive();
    }

    void StartTransmission()
    {
        Send(0x20D, 0x7, 0x0);
        int returnValue = Receive();

        //read pack untill the ack
        while ((returnValue != 1))
        {
            returnValue = Receive();
        }
    }

    void SetDataRate(int dataRate)
    {
        __u8 data_rate = (__u8)dataRate;

        Send(0x20D, 0x8, data_rate);
        int returnValue = Receive();
        //read pack untill the ack
        while ((returnValue != 1))
            returnValue = Receive();    
    }

    void GetFullScale() {
              
        __u8 B0,B1;
        int return_value;
        for (int count = 0; count < 6; ++count){
            switch (count){
                case 0: B1 = 0x0; break;    // 1566
                case 1: B1 = 0x1; break;    // 1776
                case 2: B1 = 0x2; break;    // 2122
                case 3: B1 = 0x3; break;    // 37
                case 4: B1 = 0x4; break;    // 37
                case 5: B1 = 0x5; break;    // 23
            }

            B0 = 0x18;
            Send(0x20D, B0, B1); 
            return_value = Receive();
            if (return_value < 0) 
                std::cout << "Get Full Scale Error!\n";
            scales_vect[count] = in_value;
        }

        for(int i = 0; i <6; i++)
            std::cout << i << ":" << scales_vect[i] << std::endl;
               
    }

    ~CanDevice()
    {
        if (fd >= 0)
            close(fd);
    }
    can_frame GetInFrame() {return in_frame;}

    bool IsWrenchReady() 
    {
        if(force_readed && torque_readed)
        {
            force_readed = false;
            torque_readed = false;
            return true;   
        }
        return false;
    }
    
    void GetWrench(double* wrench)
    {
        for(int i = 0; i < 3; i++)
        {
            float scale = 1.0;
            if(i == 0)
            {
                scale = 1566.0;
            }
            if(i == 1)
            {
                scale = 1776;
            }
            if(i == 2)
            {
                scale = 2122;
            }
            wrench[i] = in_force_pre_scale[i] * scale/pow(2,15) - scale;
        }

        for(int i = 3; i < 6; i++)
        {
            wrench[i] = in_torque_pre_scale[i - 3] * scales_vect[i]/pow(2,15) - scales_vect[i];
        }
        
        return;
    }
    private:

    void GetValues(unsigned short int *value){
        short int outgoing;
        outgoing = (((unsigned short )in_frame.data[2]) << 8) & 0xFF00;
        outgoing += ((unsigned short )in_frame.data[3]) & 0x00FF; 
        if(in_frame.can_id == 0x3DA)
        {
            //std::cout << "FORCE: " << outgoing << std::endl;
        }
        *value=outgoing;
    }

    bool force_readed = false;
    bool torque_readed = false;
    int fd = -1;
    uint16_t in_force_pre_scale[3];
    uint16_t in_torque_pre_scale[3];
    unsigned short scales_vect[6];
    can_frame in_frame;
    unsigned short in_value;
};

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