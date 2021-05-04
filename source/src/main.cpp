#include "StereoCalib.hpp"
#include "StereoVision.hpp"

#include <cstring>
#include <iostream>

void calibrateCameras() {
    StereoCalib sc("additional_files/left.txt", "additional_files/right.txt");
    sc.leftCameraCalibrate();
    sc.leftCameraUndistort();
    sc.rightCameraCalibrate();
    sc.rightCameraUndistort();
    sc.stereoCalibrateAndRectify();
}

void calculateDistance(std::string left, std::string right) {
    StereoVision sv(left, right);
    sv.getQMatrix();

    std::cout << "X, Y and Z coordinates of a real-world object" << std::endl;
    std::cout << sv.calculateDistance(sv.detectObject()) << std::endl;
}

void printUsage() {
    std::cout << "[USAGE] " << std::endl
              << "\tFor camera calibration: ./stereo_vision --calibrate" << std::endl
              << "\tFor distance calculation: ./stereo_vision --calculate"
                 " [TEST_PHOTO_LEFT] [TEST_PHOTO_RIGHT]" << std::endl;
}

void parseArguments(int argc, char** argv) {
    if (argc == 2
        && strcmp(argv[1], "--calibrate") == 0) {
        calibrateCameras();
    } else if (argc == 4
               && strcmp(argv[1], "--calculate") == 0) {
        calculateDistance(argv[2], argv[3]);
    } else {
        printUsage();
    }
}

int main(int argc, char** argv) {
    parseArguments(argc, argv);

    return 0;
}
