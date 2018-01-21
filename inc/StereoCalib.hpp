#ifndef __STEREO_CALIB_HPP_
#define __STEREO_CALIB_HPP_

#include <iostream>
#include <vector>
#include <stdint.h>
#include <string>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

/* Calibration checkerboard square size (in meters) */
#define SQUARE_SIZE 0.0235

class StereoCalib {
private:
    /* Checkerboard info */
    int_fast8_t m_numCornersHor;
    int_fast8_t m_numCornersVer;
    cv::Size m_boardSize;
    cv::Size m_imageSize;

    /* Images file info */
    std::string m_leftImagesFileName;
    std::string m_rightImagesFileName;
    std::ifstream m_leftImagesFilestream;
    std::ifstream m_rightImagesFilestream;

    /* Vectors of 2D and 3D points regarding the calibration */
    std::vector<std::vector<cv::Point3f>> m_objectPoints;
    std::vector<cv::Point3f> m_objectPoint;
    std::vector<std::vector<cv::Point2f>> m_imagePointsRight;
    std::vector<std::vector<cv::Point2f>> m_imagePointsLeft;
    std::vector<cv::Point2f> m_cornersRight;
    std::vector<cv::Point2f> m_cornersLeft;

    /* Intrinsic and distortion coefficients of the cameras */
    cv::Mat m_intrinsicCoeffsLeft;
    cv::Mat m_intrinsicCoeffsRight;
    cv::Mat m_distortionCoeffsLeft;
    cv::Mat m_distortionCoeffsRight;
    std::vector<cv::Mat> m_RVectorsLeft;
    std::vector<cv::Mat> m_TVectorsLeft;
    std::vector<cv::Mat> m_RVectorsRight;
    std::vector<cv::Mat> m_TVectorsRight;

    /* Output coefficients matrix */
    cv::Mat m_qMatrix;

private:
    void outputQMatrix(void);

public:
    StereoCalib(std::string leftFile, std::string rightFile);
    ~StereoCalib();
    bool leftCameraCalibrate(void);
    bool rightCameraCalibrate(void);
    bool leftCameraUndistort(void);
    bool rightCameraUndistort(void);
    bool stereoCalibrateAndRectify(void);
};

#endif
