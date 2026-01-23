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

#define PORT 10000
#define PORT_REALTIME 10001

namespace KORTEX = Kinova::Api;


int main(int argc, char **argv)
{
    KinovaLiralab::Robot* robot = new KinovaLiralab::Robot();
    //robot->GoHome();
    robot->WeightlessMode();
    /*
    auto start = std::chrono::high_resolution_clock::now();
        robot->EvaluateJacobian();
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "Execution time: " << std::chrono::duration<double>(end-start).count() << std:: endl;
    start = std::chrono::high_resolution_clock::now();
        robot->EvaluateJacobianKDL();
    end = std::chrono::high_resolution_clock::now();
    std::cout << "Execution time: " << std::chrono::duration<double>(end-start).count() << std:: endl;
    start = std::chrono::high_resolution_clock::now();
        robot->EvaluateJacobianKDLNumerically();
    end = std::chrono::high_resolution_clock::now();
    std::cout << "Execution time: " << std::chrono::duration<double>(end-start).count() << std:: endl;
    */

    // robot->VelocityControlHighLevel();
}