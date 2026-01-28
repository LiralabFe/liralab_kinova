/*
 * KINOVA (R) KORTEX (TM)
 *
 * Copyright (c) 2018 Kinova inc. All rights reserved.
 *
 * This software may be modified and distributed
 * under the terms of the BSD 3-Clause license.
 *
 * Refer to the LICENSE file for details.
 *
 */

#include <SessionManager.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>

#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>
#include <KinovaLiralab.hpp>
#include <DatasetRecorder.hpp>

#define PORT 10000
#define PORT_REALTIME 10001

namespace KORTEX = Kinova::Api;


int main(int argc, char **argv)
{
    KinovaLiralab::Robot* robot = new KinovaLiralab::Robot();
    thread realtimeThread([robot]() {robot->StartHandGuidance();});
    DatasetRecorder datasetRecorder("test", robot);
    datasetRecorder.StartRecord();
    robot->StopHandGuidance();
    realtimeThread.join();

    /*
    float timer = 0;
    while(timer < 10000)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        timer += 100;
        KinovaLiralab::RobotState robotState = robot->GetRobotState();
        std::cout << robotState._eePose.size() << std::endl;
        std::cout << robotState._eePose[0] << ", " << robotState._eePose[1] << ", " << robotState._eePose[2] << "\n";
    }
    */
}