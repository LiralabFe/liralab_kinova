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
    if(argc < 2) {std::cerr << "\nMissing argument: ['Dataset Name']\n\n"; return -1;}

    TerminationHandler t;
    KinovaLiralab::Robot* robot = new KinovaLiralab::Robot("/home/legion/ROS/kinova_ws/src/ros2_kortex/kortex_description/robots/gen3_ESAOTE_convex_probe.urdf"); // _ESAOTE_convex_probe
    DatasetRecorder datasetRecorder(static_cast<string>(argv[1]), robot);

    // Subscribe callbacks for CTRL-C signal
    TerminationHandler::RegisterCallback([&robot](){robot->StopApp();});
    TerminationHandler::RegisterCallback([&datasetRecorder](){datasetRecorder.StopRecord();});

    robot->StartHandGuidance();
    std::cin.get();
    datasetRecorder.StartRecord(400);
    // -------------------
    

    /* MODIFY Eq Pose Example: */
    //KDL::Frame eeFrame = robot->GetEEFrame();
    //eeFrame.p[0] += 0.07;
    //robot->SetEquilibriumPose(eeFrame);

    // -------------------
    //std::cin.get();
    datasetRecorder.StopRecord();
    robot->StopApp();

}