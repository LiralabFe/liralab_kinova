#pragma once

#include <string>
#include <stdio.h>
//#include <format>
#include <vector>
#include <KinovaLiralab.hpp>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/utils/filesystem.hpp>
#include <filesystem>
#include "ftSenseHandler.hpp"
#include <thread>

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
    string _recordFolder;
    std::filesystem::path _csvFilePath, _imageFilePath;
    KinovaLiralab::Robot* _robot;
    FILE* _csvFile;
    cv::VideoCapture _camera;
    cv::Rect _roi;
    std::vector<KinovaLiralab::RobotState> _robotStates;
    std::vector<double*> _forceStates;  // from force sensor
    std::vector<cv::Mat> _frames;
    std::atomic<bool> _stopRecording;
    std::thread _recordingThread;
    CanDevice* forceSensor;
    
public:
    DatasetRecorder(const std::string folderName, KinovaLiralab::Robot* robot);
    bool ContainsPureGreenPixel(const cv::Mat& frame);
    void StartRecord(int samplesNumber = -1);
    void StopRecord();
    ~DatasetRecorder();
};

DatasetRecorder::DatasetRecorder(const std::string folderName, KinovaLiralab::Robot* robot) : _recordFolder{folderName}, _robot{robot}
{
    _stopRecording = false;
    // === MAKE DIRS FOR RECORD ===
    if (!cv::utils::fs::createDirectories(folderName)) std::cerr << "Impossibile creare cartella: " << folderName << std::endl;
    if (!cv::utils::fs::createDirectories((std::filesystem::path(folderName) / "image").c_str())) std::cerr << "Impossibile creare cartella: " << _recordFolder << std::endl;

    // === CREATE CSV FILE ===
    _csvFilePath = std::filesystem::path(folderName) / (folderName + ".csv");
    _imageFilePath = std::filesystem::path(folderName) / "image";
    _csvFile = fopen(_csvFilePath.c_str(), "w");
    for (auto file : std::filesystem::directory_iterator(_imageFilePath)) // Remove all images in the directory
        std::filesystem::remove_all(file.path());

    // === FIND THE CAMERA ===
    _roi = cv::Rect(335,125, 1100-335, 600-125);
    std::cout << "VERSION :" << CV_VERSION << std::endl;

    int id;
    for(id = 0; id < 20; id++)
    {
        _camera.open(id);
        _camera.set(cv::CAP_PROP_FRAME_WIDTH,  1280);
        _camera.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
        //_camera.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));

        //cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)

        //cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        if (!_camera.isOpened())
        {
            std::cerr << "Impossibile aprire la webcam (device " << id << ")\n";
            _camera.release();
        }
        else 
        {
            std::cerr << "\n\nUsing device id " << id << "\n";
            break;
        }
    }

    // === WAIT UNTIL NO GREEN PIXELs ARE DISPLAYED ===
    cv::Mat frame;
    while(true)
    {
        _camera >> frame;
        if(ContainsPureGreenPixel(frame) == 0) break;
        _camera.release();
        std::cout << "Reopen.." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        _camera.open(id, cv::CAP_ANY);
        _camera.set(cv::CAP_PROP_FRAME_WIDTH,  1280);
        _camera.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    }

    // === OPEN CAN COMUNICATIONS ===
    forceSensor = new CanDevice();

    if(!forceSensor->Open("can0")) {"Cannot open CAN";return;}

    Eigen::Vector3d com_sensor_payload{0.0, 0.0, 0.08};
    Eigen::Isometry3d T_ee_sensor = Eigen::Isometry3d::Identity();
    Eigen::Vector3d translation(0.0, 0.0, 0.145);
    T_ee_sensor = T_ee_sensor.translate(translation);
    float payloadMass = 0.320 + 0.092;

    forceSensor->InitCompensation(com_sensor_payload, T_ee_sensor, payloadMass);

    if (!_csvFile) perror("Errore apertura file CSV");
    else
    {
        fprintf(_csvFile,
            "timestamp,"
            "q0,q1,q2,q3,q4,q5,q6,"                     // joints positions
            "v0,v1,v2,v3,v4,v5,v6,"                     // joints velocities
            "t0,t1,t2,t3,t4,t5,t6,"                     // joints torques
            "x,y,z,"                                    // ee position
            "r11,r12,r13,r21,r22,r23,r31,r32,r33,"      // ee rotation matrix
            "xForce,yForce,zForce,"                   // force sensor force
            "xTorque,yTorque,zTorque,"                // force sensor torque
            "image\n"                                   // captured frame
        );
        fflush(_csvFile);
    }
    printf("-------------\nDATASET READY\n-------------");
}


void DatasetRecorder::StartRecord(int sampleNumber)
{
    if(_recordingThread.joinable()) {std::cout << "RECORDING ALREADY RUNNING\n"; return;}
    _stopRecording = false;
    _recordingThread = std::thread([this, sampleNumber]()
    {
        /* WAIT FOR CAMERA TO OPEN */
        while(!_camera.isOpened())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            std::cout << "Waiting for camera to open...\n";
        }

        /* WAIT FOR FIRST CAN MSG */
        while (!forceSensor->IsWrenchReady())
        {
            forceSensor->Receive();
            std::cout << "Waiting for can msgs...\n";
        }
        

        using clock = std::chrono::high_resolution_clock;
        int remainingSample = sampleNumber;
        _robotStates.clear();
        _forceStates.clear();
        std::vector<double> timestamps;

        auto t0 = clock::now();

        int recordedFrames = 0;
        int32_t sampleTime = 100; // millis
        double wrench[6];
        

        while ((sampleNumber > 0 && remainingSample > 0) || (sampleNumber <= 0 && !_stopRecording))
        {
            if(sampleNumber > 0) {remainingSample--; std::cout << "Remaining samples: " << remainingSample << " \n";};

            // === FRAME CAMERA ===
            cv::Mat frame;
            cv::Mat gray;

            _camera >> frame;   // cattura frame

            if (frame.empty()) continue;

            frame = frame(_roi);
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
            cv::normalize(gray, gray, 0, 255, cv::NORM_MINMAX);
            //_frames.push_back(gray.clone());
            
            std::string name = "img_" + std::to_string(recordedFrames) + ".png";
            auto path = _imageFilePath / name;

            cv::imwrite(path.string(), frame);

            if(recordedFrames == 0)
                forceSensor->SetTare();

            KinovaLiralab::RobotState newState = _robot->GetRobotState();
            forceSensor->ReceiveAllForceAndTorque();
            forceSensor->GetWrenchCompensated(wrench, newState);

            auto now = clock::now();
            double timestamp = std::chrono::duration<double>(now - t0).count();

            _robotStates.push_back(newState);
            _forceStates.push_back(wrench);
            timestamps.push_back(timestamp);

            /* Wait sample time and update CAN read in the meantime. Maybe is not usefull */
            std::this_thread::sleep_for(std::chrono::milliseconds(sampleTime));
            //for(int i = 0; i < 2; i++) forceSensor->Receive();

            recordedFrames++;
        }

        std::cout << "Recording fermato. Scrittura CSV..." << std::endl;

        // === SCRITTURA CSV ===
        std::cout << "Robot States: " << _robotStates.size() << std::endl;
        std::cout << "Force sensor states: " << _forceStates.size() << std::endl;
        std::cout << "Frames: " << _frames.size() << std::endl;

        for (size_t i = 0; i < _robotStates.size(); ++i)
        {
            const KinovaLiralab::RobotState& s = _robotStates[i];
            const double* f = _forceStates[i];

            fprintf(_csvFile,
                "%.6f,"
                "%f,%f,%f,%f,%f,%f,%f,"             // q0-q6
                "%f,%f,%f,%f,%f,%f,%f,"             // v0-v6
                "%f,%f,%f,%f,%f,%f,%f,"             // t0-t6
                "%f,%f,%f,"                         // x y z
                "%f,%f,%f,%f,%f,%f,%f,%f,%f,"       // R 3x3
                "%f,%f,%f,"                         // force sensor [FORCE]
                "%f,%f,%f,"                         // force sensor [TORQUE]
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
                f[0], f[1], f[2],
                f[3], f[4], f[5],
                ("img_" + std::to_string(i) + ".png").c_str()
            );
        }

        /*
        for (size_t i = 0; i < _frames.size(); ++i)
        {
            std::string name = "img_" + std::to_string(i) + ".png";
            auto path = _imageFilePath / name;
            std::cout << "Saving " << name << "\n";
            cv::imwrite(path.string(), _frames[i]);
        }
        */

        fflush(_csvFile);

        std::cout << "CSV scritto: " << _csvFilePath.c_str() << "\n";
        std::cout << "Samples registrati: " << _robotStates.size() << "\n";
    });
}

bool DatasetRecorder::ContainsPureGreenPixel(const cv::Mat& frame)
{
    cv::Mat mask;
    cv::inRange(
        frame,
        cv::Scalar(0, 100, 0),
        cv::Scalar(0, 255, 0),
        mask);

    return cv::countNonZero(mask) > 0;
}

void DatasetRecorder::StopRecord()
{
    _stopRecording = true;
    _recordingThread.join();
}
DatasetRecorder::~DatasetRecorder()
{
    if (_csvFile) fclose(_csvFile);
}

