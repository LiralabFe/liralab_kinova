#include <SessionManager.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>

#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>
#include <KinovaLiralab.hpp>
#include <DatasetRecorder.hpp>
#include <TerminationHandler.hpp>
#include <SocketLiralab.hpp>
#include <AurovasSocket.hpp>
#include <thread>
#include <chrono>

#define PORT 10000
#define PORT_REALTIME 10001

namespace KORTEX = Kinova::Api;


int main(int argc, char **argv)
{   
    // Uncomment the application

    /* ******************************************** */
    /* *********** REGISTER NEW EPISODES ********** */
    /* ******************************************** */
    /*
    if(argc < 2) {std::cerr << "\nMissing argument: ['Dataset Name']\n\n"; return -1;}

    TerminationHandler t;
    KinovaLiralab::Robot* robot = new KinovaLiralab::Robot("/home/legion/ROS/kinova_ws/src/ros2_kortex/kortex_description/robots/gen3_ESAOTE_convex_probe.urdf"); // _ESAOTE_convex_probe
    DatasetRecorder datasetRecorder(static_cast<string>(argv[1]), robot);

    // Subscribe callbacks for CTRL-C signal
    TerminationHandler::RegisterCallback([&robot](){robot->StopApp();});
    TerminationHandler::RegisterCallback([&datasetRecorder](){datasetRecorder.StopRecord();});

    robot->StartHandGuidance();
    std::cin.get();
    datasetRecorder.StartRecord(600);   
    std::cin.get();
    datasetRecorder.StopRecord();
    robot->StopApp();
    
    */

    /* ******************************************** */
    /* **************** RUN ACT ******************* */
    /* ******************************************** */
    
    /*
    TerminationHandler t;
    KinovaLiralab::Robot* robot = new KinovaLiralab::Robot("/home/legion/ROS/kinova_ws/src/ros2_kortex/kortex_description/robots/gen3_ESAOTE_convex_probe.urdf"); // _ESAOTE_convex_probe
    KinovaLiralab::SocketLiralab socket{5000, [&robot]{robot->StopApp();}};

    // Subscribe callbacks for CTRL-C signal
    TerminationHandler::RegisterCallback([&robot](){robot->StopApp();});
    TerminationHandler::RegisterCallback([&socket](){socket.CloseSocket();});

    std::cout << "Position the probe on belly and press ENTER" << std::endl;
    robot->StartHandGuidance();
    std::cin.get();

    // ---------- Send initial position
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    KinovaLiralab::RobotState state = robot->GetRobotState();
    socket.WriteRobotState(state);

    // ---------- Wait acknoledge from python
    while(socket.Read() != "RUN");
    robot->StopApp();
    robot->TorqueControl();

    KDL::Frame newFrame{};
    while(true)
    {
        KinovaLiralab::RobotState state = robot->GetRobotState();
        socket.WriteRobotState(state);
        
        if(socket.ReadFrame(newFrame) < 0) break;

        robot->SetEquilibriumPose(newFrame);
    }
    
    std::cin.get();
    robot->StopApp();
    */
    /* ******************************************** */
    /* **************** AUROVAS KINOVA ************ */
    /* ******************************************** */
    
    TerminationHandler t;
    AurovasSocket socket;
    KinovaLiralab::Robot* robot = new KinovaLiralab::Robot("/home/legion/ROS/kinova_ws/src/ros2_kortex/kortex_description/robots/gen3_ESAOTE_convex_probe.urdf"); // _ESAOTE_convex_probe

    TerminationHandler::RegisterCallback([&robot](){robot->StopApp();});
    TerminationHandler::RegisterCallback([&socket](){socket.CloseSocket();});

    std::vector<std::string> message = socket.Read();
    socket.Write("WAITING;STARTING");
    socket.Write("STARTING;WAITING");

    socket.Read();  // HANDGUIDING;POSITION
    robot->StartHandGuidance();
    std::cin.get();
    robot->StopApp();
    std::vector<string> msg = socket.Read();  // GLOBALSEARCH;5,5,8,8,35,10
    int row = std::stoi(msg[1]);
    int col = std::stoi(msg[2]);
    int dist = std::stoi(msg[3]);
    int x = 0;
    int dir = 1; // 1 = avanti, -1 = indietro

    socket.Write("HANDGUIDING;WAITING");
    socket.Write("WAITING;GLOBALSEARCH");

    robot->TorqueControl();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    KDL::Frame eeInitial = robot->GetEEFrame();
    for(int y = 0; y < col; y++)
    {
        for(int step = 0; step < row; step++)
        {
            std::cout << "Moving " << x << ", " << y << std::endl;

            KDL::Frame eeFrame = eeInitial;
            eeFrame.p[0] -= dist/100.0 * x;
            eeFrame.p[1] += dist/100.0 * y;

            robot->SetEquilibriumPose(eeFrame);
            std::cout << "Segmentation" << std::endl;

            std::this_thread::sleep_for(std::chrono::seconds(2));
            std::cin.get();
            if(socket.Read()[0] == "True") break;
            socket.Write("TEST;TEST");       
            if(step < 4)
                x += dir;
        }

        dir *= -1;
    }
    
    socket.Write("GLOBALSEARCH;LOCALSEARCH");
    // LOCAL SEARCH
    socket.Read();
    socket.Write("LOCALSEARCH;WAITING");
    
    robot->StopApp();

    robot->StopApp();

    
    // Subscribe callbacks for CTRL-C signal
    //TerminationHandler::RegisterCallback([&robot](){robot->StopApp();});

    //std::cout << "EXAMPLE" << std::endl;
    //std::cin.get();
    //robot->StartHandGuidance();
    //std::cin.get();
    //robot->StopApp();
    //robot->TorqueControl();
    //KDL::Frame eeFrame = robot->GetEEFrame();
    //eeFrame.p[0] += 0.07;
    //robot->SetEquilibriumPose(eeFrame);
}