#ifndef DATASET_RECORDER
#define DATASET_RECORDER

#include <string>
#include <stdio.h>
#include <format>
#include <vector>
#include <KinovaLiralab.hpp>
#include <chrono>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/utils/filesystem.hpp>
#include <filesystem>
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
    std::vector<cv::Mat> _frames;
    std::atomic<bool> _stopRecording;
    std::thread _recordingThread;
    
public:
    DatasetRecorder(const std::string folderName, KinovaLiralab::Robot* robot);
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

    // 
    _csvFilePath = std::filesystem::path(folderName) / (folderName + ".csv");
    _imageFilePath = std::filesystem::path(folderName) / "image";
    _csvFile = fopen(_csvFilePath.c_str(), "w");
    for (auto file : std::filesystem::directory_iterator(_imageFilePath)) // Remove all images in the directory
        std::filesystem::remove_all(file.path());

    int deviceID = 2;   // 0 = webcam default
    _camera.open(deviceID);
    _camera.set(cv::CAP_PROP_FRAME_WIDTH,  1280);
    _camera.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    _roi = cv::Rect(335,125, 1100-335, 600-125);

    if (!_camera.isOpened()) std::cerr << "Impossibile aprire la webcam (device " << deviceID << ")\n";

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


void DatasetRecorder::StartRecord(int sampleNumber)
{
    if(_recordingThread.joinable()) {std::cout << "RECORDING ALREADY RUNNING\n"; return;}
    _stopRecording = false;
    _recordingThread = std::thread([this, sampleNumber]()
    {
        using clock = std::chrono::high_resolution_clock;
        int remainingSample = sampleNumber;
        _robotStates.clear();
        std::vector<double> timestamps;

        auto t0 = clock::now();

        int recordedFrames = 0;
        while ((sampleNumber > 0 && remainingSample > 0) || (sampleNumber <= 0 && !_stopRecording))
        {
            if(sampleNumber > 0) {remainingSample--; std::cout << "Remaining samples: " << remainingSample << " \n";};
            // === FRAME CAMERA ===
            cv::Mat frame, gray;

            _camera >> frame;   // cattura frame

            if (frame.empty()) continue;

            frame = frame(_roi);
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
            cv::normalize(gray, gray, 0, 255, cv::NORM_MINMAX);
            //_frames.push_back(gray.clone());
            
            std::string name = "img_" + std::to_string(recordedFrames) + ".png";
            auto path = _imageFilePath / name;
            std::cout << "Saving " << name << "\n";
            cv::imwrite(path.string(), frame);

            KinovaLiralab::RobotState newState = _robot->GetRobotState();

            auto now = clock::now();
            double timestamp = std::chrono::duration<double>(now - t0).count();

            _robotStates.push_back(newState);
            timestamps.push_back(timestamp);

            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // <-------------------------------------------------------------------- SAMPLE TIME !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            recordedFrames++;
            if((recordedFrames % 10) == 0) std::cout << "Recorded: " << recordedFrames << std::endl;
        }

        std::cout << "Recording fermato. Scrittura CSV..." << std::endl;

        // === SCRITTURA CSV ===
        std::cout << "Robot States: " << _robotStates.size() << std::endl;
        std::cout << "Frames: " << _frames.size() << std::endl;

        for (size_t i = 0; i < _robotStates.size(); ++i)
        {
            const KinovaLiralab::RobotState& s = _robotStates[i];

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

void DatasetRecorder::StopRecord()
{
    _stopRecording = true;
    _recordingThread.join();
}
DatasetRecorder::~DatasetRecorder()
{
    if (_csvFile) fclose(_csvFile);
}


#endif