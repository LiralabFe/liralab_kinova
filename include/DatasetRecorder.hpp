#ifndef DATASET_RECORDER
#define DATASET_RECORDER

#include <string>
#include <stdio.h>
#include <format>
#include <vector>
#include <KinovaLiralab.hpp>
#include <chrono>
#include <opencv2/core.hpp>
#include <opencv2/core/utils/filesystem.hpp>
#include <filesystem>

#ifdef _WIN32
#include <conio.h>   // Windows
#else
#include <unistd.h>
#include <termios.h> // Linux
#include <fcntl.h>
#endif

class DatasetRecorder
{
private:
    string _csvName, _recordFolder;
    std::filesystem::path _csvFilePath, _imageFilePath;
    KinovaLiralab::Robot* _robot;
    std::vector<KinovaLiralab::RobotState> _robotStates;
    FILE* _csvFile;

public:
    DatasetRecorder(const std::string folderName, KinovaLiralab::Robot* robot);
    void StartRecord();
    ~DatasetRecorder();
};

DatasetRecorder::DatasetRecorder(const std::string folderName, KinovaLiralab::Robot* robot) : _recordFolder{folderName}, _robot{robot}
{
    if (!cv::utils::fs::createDirectories(folderName)) std::cerr << "Impossibile creare cartella: " << folderName << std::endl;
    if (!cv::utils::fs::createDirectories((std::filesystem::path(folderName) / "image").c_str())) std::cerr << "Impossibile creare cartella: " << _recordFolder << std::endl;

    _csvFilePath = std::filesystem::path(folderName) / (folderName + ".csv");
    _imageFilePath = std::filesystem::path(folderName) / "image";
    std::cout << "FILE PATH: " << _csvFilePath.c_str() << std::endl;
    std::cout << "IMAGE PATH: " << _imageFilePath.c_str() << std::endl;
    _csvName = _csvFilePath.c_str();
    _csvFile = fopen(_csvName.c_str(), "w");

    if (!_csvFile) perror("Errore apertura file CSV");
    else
    {
        fprintf(_csvFile,
            "timestamp,"
            "q0,q1,q2,q3,q4,q5,q6,"
            "v0,v1,v2,v3,v4,v5,v6,"
            "t0,t1,t2,t3,t4,t5,t6,"
            "x,y,z,"
            "r11,r12,r13,r21,r22,r23,r31,r32,r33,"
            "image\n"
        );
        fflush(_csvFile);
    }
}


void DatasetRecorder::StartRecord()
{
    using clock = std::chrono::high_resolution_clock;

    _robotStates.clear();
    std::vector<double> timestamps;
    std::atomic<bool> stop(false);

    std::cout << "Recording... premi INVIO per fermare\n";
    std::thread inputThread([&stop]() {
        std::cin.get();
        stop = true;
    });

    auto t0 = clock::now();

    while (!stop)
    {
        KinovaLiralab::RobotState newState = _robot->GetRobotState();

        auto now = clock::now();
        double timestamp = std::chrono::duration<double>(now - t0).count();

        _robotStates.push_back(newState);
        timestamps.push_back(timestamp);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    inputThread.join();
    std::cout << "Recording fermato. Scrittura CSV...\n";

    // === SCRITTURA CSV ===
    for (size_t i = 0; i < _robotStates.size(); ++i)
    {
        const auto& s = _robotStates[i];

        fprintf(_csvFile,
            "%.6f,"
            "%f,%f,%f,%f,%f,%f,%f,"             // q0-q6
            "%f,%f,%f,%f,%f,%f,%f,"             // v0-v6
            "%f,%f,%f,%f,%f,%f,%f,"             // t0-t6
            "%f,%f,%f,"                         // x y z
            "%f,%f,%f,%f,%f,%f,%f,%f,%f,"       // R 3x3
            "%s\n",                             // ultrasound image
            timestamps[i],
            s._jointPositions[0], s._jointPositions[1], s._jointPositions[2],
            s._jointPositions[3], s._jointPositions[4], s._jointPositions[5], s._jointPositions[6],
            s._jointVels[0], s._jointVels[1], s._jointVels[2], s._jointVels[3],
            s._jointVels[4], s._jointVels[5], s._jointVels[6],
            s._jointTorques[0], s._jointTorques[1], s._jointTorques[2], s._jointTorques[3],
            s._jointTorques[4], s._jointTorques[5], s._jointTorques[6],
            s._eePose[0], s._eePose[1], s._eePose[2],
            s._eePose[3], s._eePose[4], s._eePose[5],
            s._eePose[6], s._eePose[7], s._eePose[8],
            s._eePose[9], s._eePose[10], s._eePose[11],
            "echo.png"
        );
    }

    fflush(_csvFile);

    std::cout << "CSV scritto: " << _csvName << "\n";
    std::cout << "Samples registrati: " << _robotStates.size() << "\n";
}

DatasetRecorder::~DatasetRecorder()
{
    if (_csvFile) fclose(_csvFile);
}


#endif