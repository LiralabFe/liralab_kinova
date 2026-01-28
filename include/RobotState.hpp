#ifndef ROBOT_STATE
#define ROBOT_STATE

#include <vector>

namespace KinovaLiralab
{
    struct RobotState
    {
        std::vector<float> _jointPositions; // size 7
        std::vector<float> _jointVels;      // size 7
        std::vector<float> _jointTorques;   // size 7
        std::vector<float> _eePose;         // size 12
    };
}

#endif