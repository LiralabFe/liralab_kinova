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
#include <TerminationHandler.hpp>

#define PORT 10000
#define PORT_REALTIME 10001

namespace KORTEX = Kinova::Api;


int main(int argc, char **argv)
{    
    TerminationHandler t;
    KinovaLiralab::Robot* robot = new KinovaLiralab::Robot();
    DatasetRecorder datasetRecorder("test", robot);

    // Subscribe callbacks for CTRL-C signal
    TerminationHandler::RegisterCallback([&robot](){robot->StopHandGuidance();});
    TerminationHandler::RegisterCallback([&datasetRecorder](){datasetRecorder.StopRecord();});

    robot->StartHandGuidance();
    datasetRecorder.StartRecord();
    std::cin.get();
    datasetRecorder.StopRecord();
    robot->StopHandGuidance();

}