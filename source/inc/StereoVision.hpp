#ifndef __STEREO_VISION_HPP_
#define __STEREO_VISION_HPP_

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

typedef enum { LEFT, RIGHT } left_right;

class StereoVision {
private:
    /* Q Matrix */
    cv::Mat m_qMatrix;

    std::string m_leftImageName;
    std::string m_rightImageName;

    /* Matrices containing left and right camera images */
    cv::Mat m_leftFrame;
    cv::Mat m_rightFrame;
    cv::Mat m_hsvLeftFrame;
    cv::Mat m_hsvRightFrame;
    cv::Mat m_leftThresholdFrame;
    cv::Mat m_rightThresholdFrame;

    /* Rectangle detection objects */
    std::vector<std::vector<cv::Point>> m_contoursLeft;
    std::vector<std::vector<cv::Point>> m_contoursRight;
    std::vector<cv::Vec4i> m_hierarchyLeft;
    std::vector<cv::Vec4i> m_hierarchyRight;
    std::vector<cv::Rect> m_rectanglesLeft;
    std::vector<cv::Rect> m_rectanglesRight;
    cv::Rect m_maxRectLeft;
    cv::Rect m_maxRectRight;

private:
    void findRectangles(left_right lr);
    void convertColourSpaces(left_right lr);
public:
    StereoVision() = default;
    StereoVision(std::string leftImage, std::string rightImage);
    void setImageNames(std::string leftImage, std::string rightImage);
    cv::Point3f calculateDistance(cv::Point3f in);
    cv::Point3f detectObject(void);
    void getQMatrix(void);
};

#endif
