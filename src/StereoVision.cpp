#include "../inc/StereoVision.hpp"
#include <fstream>
#include <iostream>
#include <stdint.h>

StereoVision::StereoVision(std::string leftImage, std::string rightImage) {
    m_leftImageName.assign(leftImage);
    m_rightImageName.assign(rightImage);
}

void StereoVision::setImageNames(std::string leftImage, std::string rightImage) {
    m_leftImageName.assign(leftImage);
    m_rightImageName.assign(rightImage);
}

void StereoVision::getQMatrix(void) {
    std::ifstream qMatrixStream;

    /* Open Q matrix file, provided by StereoCalibration class */
    qMatrixStream.open("additional_files/q_matrix");
    m_qMatrix = cv::Mat(4, 4, CV_64F);

    /* Extract the Q matrix */
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            qMatrixStream >> m_qMatrix.at<double>(i, j);
        }
    }

    std::cout << m_qMatrix << std::endl;

    qMatrixStream.close();
}

void StereoVision::convertColourSpaces(left_right lr) {
    /* HSV thresholds for colour */
    cv::Scalar min(90, 100, 100);
    cv::Scalar max(120, 255, 255);
    cv::Mat strEl = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    switch (lr) {
    case LEFT:
        cv::cvtColor(m_leftFrame, m_hsvLeftFrame, CV_BGR2HSV);
        cv::inRange(m_hsvLeftFrame, min, max, m_leftThresholdFrame);

        morphologyEx(m_leftThresholdFrame, m_leftThresholdFrame, cv::MORPH_OPEN, strEl);
        morphologyEx(m_leftThresholdFrame, m_leftThresholdFrame, cv::MORPH_CLOSE, strEl);

        break;
    case RIGHT:
        cv::cvtColor(m_rightFrame, m_hsvRightFrame, CV_BGR2HSV);
        cv::inRange(m_hsvRightFrame, min, max, m_rightThresholdFrame);

        morphologyEx(m_rightThresholdFrame, m_rightThresholdFrame, cv::MORPH_OPEN, strEl);
        morphologyEx(m_rightThresholdFrame, m_rightThresholdFrame, cv::MORPH_CLOSE, strEl);

        break;
    }
}

void StereoVision::findRectangles(left_right lr) {
    int_fast32_t count;
    cv::RotatedRect rotRect;
    cv::Rect rect;

    switch (lr) {
    case LEFT:
        cv::findContours(m_leftThresholdFrame.clone(), m_contoursLeft, m_hierarchyLeft,
                CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

        count = m_contoursLeft.size();
        for (int i = 0; i < count; i++) {
            rotRect = cv::minAreaRect(m_contoursLeft[i]);
            rect = rotRect.boundingRect();

            m_rectanglesLeft.push_back(rect);
        }

        m_maxRectLeft = *m_rectanglesLeft.begin();
        for (auto it = m_rectanglesLeft.begin(); it != m_rectanglesLeft.end(); ++it) {
            if ((*it).height * (*it).width > m_maxRectLeft.height * m_maxRectLeft.width) {
                m_maxRectLeft = *it;
            }
        }

        break;
    case RIGHT:
        cv::findContours(m_rightThresholdFrame.clone(), m_contoursRight, m_hierarchyRight,
                CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

        count = m_contoursRight.size();
        for (int i = 0; i < count; i++) {
            rotRect = cv::minAreaRect(m_contoursRight[i]);
            rect = rotRect.boundingRect();

            m_rectanglesRight.push_back(rect);
        }

        m_maxRectRight = *m_rectanglesRight.begin();
        for (auto it = m_rectanglesRight.begin(); it != m_rectanglesRight.end(); ++it) {
            if ((*it).height * (*it).width > m_maxRectRight.height * m_maxRectRight.width) {
                m_maxRectRight = *it;
            }
        }

        break;
    }
}

cv::Point3f StereoVision::detectObject(void) {
    cv::Scalar red(255, 0, 0);

    m_leftFrame = cv::imread(m_leftImageName);
    m_rightFrame = cv::imread(m_rightImageName);

    /* Convert images to HSV and filter for colour */
    convertColourSpaces(LEFT);
    convertColourSpaces(RIGHT);

    /* Find the biggest rectangle on both images */
    findRectangles(LEFT);
    findRectangles(RIGHT);

    /* Draw the rectangles */
    cv::rectangle(m_rightThresholdFrame, m_maxRectRight, red, 3, 8, 0);
    cv::rectangle(m_leftThresholdFrame, m_maxRectLeft, red, 3, 8, 0);

    cv::imshow("Left", m_leftThresholdFrame);
    cv::imshow("Right", m_rightThresholdFrame);
    cv::waitKey(0);

    return cv::Point3f(m_maxRectLeft.x, m_maxRectLeft.y, abs(m_maxRectLeft.x - m_maxRectRight.x));
}

cv::Point3f StereoVision::calculateDistance(cv::Point3f in) {
    std::vector<cv::Point3f> input;
    std::vector<cv::Point3f> output;

    input.push_back(in);

    /* Transform 2D coordinates to 3D coordinates, using Q matrix */
    cv::perspectiveTransform(input, output, m_qMatrix);

    return output[0];
}
