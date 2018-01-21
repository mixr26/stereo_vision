#include "../inc/StereoCalib.hpp"

StereoCalib::StereoCalib(std::string leftFile, std::string rightFile) {
    /* Open the files containing image names */
    m_leftImagesFileName.assign(leftFile);
    m_rightImagesFileName.assign(rightFile);

    m_leftImagesFilestream.open(m_leftImagesFileName);
    m_rightImagesFilestream.open(m_rightImagesFileName);

    /* Board size */
    m_numCornersHor = 9;
    m_numCornersVer = 7;
    m_boardSize = cv::Size(m_numCornersHor, m_numCornersVer);

    /* Initialize intrinsic coeffs matrices */
    m_intrinsicCoeffsLeft = cv::Mat(3, 3, CV_64F);
    m_intrinsicCoeffsRight = cv::Mat(3, 3, CV_64F);

    m_intrinsicCoeffsLeft.ptr<float>(0)[0] = 1;
    m_intrinsicCoeffsRight.ptr<float>(0)[0] = 1;

    /* Populate 2D points matrix */
    for (int i = 0; i < m_numCornersVer; i++) {
        for (int j = 0; j < m_numCornersHor; j++) {
            m_objectPoint.push_back(cv::Point3f(float(j * SQUARE_SIZE), float(i * SQUARE_SIZE), 0.0f));
        }
    }
}

StereoCalib::~StereoCalib() {
    m_leftImagesFilestream.close();
    m_rightImagesFilestream.close();
}

bool StereoCalib::leftCameraCalibrate(void) {
    cv::Mat image;
    cv::Mat grayImage;
    std::string imageName;

    m_objectPoints.clear();

    while (!m_leftImagesFilestream.eof()) {
        bool found;

        /* Load image into the matrix */
        m_leftImagesFilestream >> imageName;

        image = cv::imread(imageName.c_str());
        if (!image.empty()) {
            cv::cvtColor(image, grayImage, CV_BGR2GRAY);
        } else {
            std::cout << "Error reading image: " << imageName << std::endl;
            return false;
        }

        /* Check for chessboard corners */
        found = cv::findChessboardCorners(image, m_boardSize, m_cornersLeft, CV_CALIB_CB_ADAPTIVE_THRESH
                | CV_CALIB_CB_FILTER_QUADS | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

        if (found) {
            std::cout << "Checkerboard detected on image: " << imageName << std::endl;

            cv::cornerSubPix(grayImage, m_cornersLeft, cv::Size(11, 11), cv::Size(-1, -1),
                    cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
            cv::drawChessboardCorners(grayImage, m_boardSize, m_cornersLeft, found);

            m_imageSize = image.size();
            m_imagePointsLeft.push_back(m_cornersLeft);
            m_objectPoints.push_back(m_objectPoint);
        } else {
            std::cout << "Checkerboard NOT detected on image: " << imageName << std::endl;
            return false;
        }
    }

    /* Calibrate using openCV function */
    cv::calibrateCamera(m_objectPoints, m_imagePointsLeft, image.size(), m_intrinsicCoeffsLeft,
            m_distortionCoeffsLeft, m_RVectorsLeft, m_TVectorsLeft);
    std::cout << "Focal length left: " << m_intrinsicCoeffsLeft.at<double>(0, 0) << std::endl;

    return true;
}

bool StereoCalib::rightCameraCalibrate(void) {
    cv::Mat image;
    cv::Mat grayImage;
    std::string imageName;

    m_objectPoints.clear();

    while (!m_rightImagesFilestream.eof()) {
        bool found;

        /* Load image into the matrix */
        m_rightImagesFilestream >> imageName;

        image = cv::imread(imageName.c_str());
        if (!image.empty()) {
            cv::cvtColor(image, grayImage, CV_BGR2GRAY);
        } else {
            std::cout << "Error reading image: " << imageName << std::endl;
            return false;
        }

        /* Check for chessboard corners */
        found = cv::findChessboardCorners(image, m_boardSize, m_cornersRight, CV_CALIB_CB_ADAPTIVE_THRESH
                | CV_CALIB_CB_FILTER_QUADS | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

        if (found) {
            std::cout << "Checkerboard detected on image: " << imageName << std::endl;

            cv::cornerSubPix(grayImage, m_cornersRight, cv::Size(11, 11), cv::Size(-1, -1),
                    cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
            cv::drawChessboardCorners(grayImage, m_boardSize, m_cornersRight, found);

            m_imageSize = image.size();
            m_imagePointsRight.push_back(m_cornersRight);
            m_objectPoints.push_back(m_objectPoint);
        } else {
            std::cout << "Checkerboard NOT detected on image: " << imageName << std::endl;
            return false;
        }
    }

    /* Calibrate using openCV function */
    cv::calibrateCamera(m_objectPoints, m_imagePointsRight, image.size(), m_intrinsicCoeffsRight,
            m_distortionCoeffsRight, m_RVectorsRight, m_TVectorsRight);
    std::cout << "Focal length right: " << m_intrinsicCoeffsRight.at<double>(0, 0) << std::endl;

    return true;
}

bool StereoCalib::leftCameraUndistort(void) {
    cv::Mat distortedImage;
    cv::Mat undistortedImage;
    std::string imageName;

    /* Get the first image*/
    m_leftImagesFilestream.clear();
    m_leftImagesFilestream.seekg(0);
    m_leftImagesFilestream >> imageName;
    distortedImage = cv::imread(imageName.c_str());

    if (distortedImage.empty()) {
        return false;
    }
    /* Undistort the image */
    cv::undistort(distortedImage, undistortedImage, m_intrinsicCoeffsLeft, m_distortionCoeffsLeft);

    /* Show both distorted and undistorted */
    cv::imshow("Distorted", distortedImage);
    cv::imshow("Undistorted", undistortedImage);
    cv::waitKey(0);

    return true;
}

bool StereoCalib::rightCameraUndistort(void) {
    cv::Mat distortedImage;
    cv::Mat undistortedImage;
    std::string imageName;

    /* Get the first image*/
    m_rightImagesFilestream.clear();
    m_rightImagesFilestream.seekg(0);
    m_rightImagesFilestream >> imageName;
    distortedImage = cv::imread(imageName.c_str());
    if (distortedImage.empty()) {
        return false;
    }

    /* Undistort the image */
    cv::undistort(distortedImage, undistortedImage, m_intrinsicCoeffsRight, m_distortionCoeffsRight);

    /* Show both distorted and undistorted */
    cv::imshow("Distorted", distortedImage);
    cv::imshow("Undistorted", undistortedImage);
    cv::waitKey(0);

    return true;
}

bool StereoCalib::stereoCalibrateAndRectify(void) {
    /* Stereo calibration parameters */
    cv::Mat R = cv::Mat(3, 3, CV_64F);
    cv::Mat T = cv::Mat(3, 1, CV_64F);
    cv::Mat E = cv::Mat(3, 3, CV_64F);
    cv::Mat F = cv::Mat(3, 3, CV_64F);

    /* Stereo rectification parameters */
    cv::Mat R1 = cv::Mat(3, 3, CV_64F);
    cv::Mat R2 = cv::Mat(3, 3, CV_64F);
    cv::Mat P1 = cv::Mat(3, 4, CV_64F);
    cv::Mat P2 = cv::Mat(3, 4, CV_64F);
    m_qMatrix = cv::Mat(4, 4, CV_64F);

    cv::stereoCalibrate(m_objectPoints, m_imagePointsLeft, m_imagePointsRight, m_intrinsicCoeffsLeft,
            m_distortionCoeffsLeft, m_intrinsicCoeffsRight, m_distortionCoeffsRight, m_imageSize,
            R, T, E, F, CV_CALIB_FIX_INTRINSIC,
            cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 100, 1e-5));

    cv::stereoRectify(m_intrinsicCoeffsLeft, m_distortionCoeffsLeft, m_intrinsicCoeffsRight,
            m_distortionCoeffsRight, m_imageSize, R, T, R1, R2, P1, P2, m_qMatrix, CV_CALIB_ZERO_DISPARITY);

    outputQMatrix();

    return true;
}

void StereoCalib::outputQMatrix(void) {
    std::ofstream qOutput;

    /* Open Q matrix output file */
    qOutput.open("additional_files/q_matrix");

    /* Write Q matrix to file */
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            qOutput << m_qMatrix.at<double>(i, j) << " ";
        }
        qOutput << std::endl;
    }

    qOutput.close();
}
