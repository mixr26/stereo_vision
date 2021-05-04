#include "../inc/StereoCalib.hpp"
#include "../inc/StereoVision.hpp"
#include <iostream>

int main() {
    /*
    StereoCalib sc("additional_files/left.txt", "additional_files/right.txt");
    sc.leftCameraCalibrate();
    sc.leftCameraUndistort();
    sc.rightCameraCalibrate();
    sc.rightCameraUndistort();
    sc.stereoCalibrateAndRectify();
    */

    StereoVision sv("test_photos/left-1.jpeg", "test_photos/right-1.jpg");
    sv.getQMatrix();

    std::cout << "X, Y and Z coordinates of a real-world object" << std::endl;
    std::cout << sv.calculateDistance(sv.detectObject()) << std::endl;
}
