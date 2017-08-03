#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

//square size in millimeters
#define SQUARE_SIZE 0.0235

int main(int argc, char** argv) {
	int num_boards = 14;
	int num_corners_hor = 9;
	int num_corners_ver = 7;
	int num_squares = num_corners_ver * num_corners_hor;
	Size board_size = Size(num_corners_hor, num_corners_ver);

	char image_name_left[13];
	char image_name_right[14];
	char* filename_left = argv[1];
	char* filename_right = argv[2];
	FILE* file_left = fopen("left.txt", "r");
	FILE* file_right = fopen("right.txt", "r");

	std::vector<std::vector<Point3f>> object_points;
	std::vector<std::vector<Point2f>> image_points_left;
	std::vector<std::vector<Point2f>> image_points_right;
	std::vector<Point2f> corners_left;
	std::vector<Point2f> corners_right;

	Mat image;
	Mat gray_image;
	Mat intrinsic_left = Mat(3, 3, CV_64F);
	Mat intrinsic_right = Mat(3, 3, CV_64F);
	Mat dist_coeffs_left;
	Mat dist_coeffs_right;
	std::vector<Mat> rvecs_left;
	std::vector<Mat> tvecs_left;
	std::vector<Mat> rvecs_right;
	std::vector<Mat> tvecs_right;

	intrinsic_left.ptr<float>(0)[0] = 1;
	intrinsic_right.ptr<float>(0)[0] = 1;

	std::vector<Point3f> obj;

	for (int i = 0; i < num_corners_ver; i++) {
		for (int j = 0; j < num_corners_hor; j++) {
			obj.push_back(Point3f(float(j * SQUARE_SIZE), float(i * SQUARE_SIZE), 0.0f));
		}
	}

	//LEFT CAMERA CALIBRATION
	while (fscanf(file_left, "%s", image_name_left) != EOF) {
		image = imread(image_name_left);
		if (!image.empty())
			cvtColor(image, gray_image, CV_BGR2GRAY);

		bool found = findChessboardCorners(image, board_size, corners_left, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
		if (found) {
			cornerSubPix(gray_image, corners_left, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			drawChessboardCorners(gray_image, board_size, corners_left, found);

			image_points_left.push_back(corners_left);
			object_points.push_back(obj);
		}
	}

	calibrateCamera(object_points, image_points_left, image.size(), intrinsic_left, dist_coeffs_left, rvecs_left, tvecs_left);

	std::cout << "focal length left: " << intrinsic_left.at<double>(0, 0) << std::endl;
/*
	//REMOVE DISTORTION - TEST
	
	Mat img, img_undist;
	img = imread("left-1.jpeg");
	undistort(img, img_undist, intrinsic_left, dist_coeffs_left);
	imshow("dist", img);
	imshow("undist", img_undist);
	waitKey(0);
*/		
	//RIGHT CAMERA CALIBRATION
	while (fscanf(file_right, "%s", image_name_right) != EOF) {
		image = imread(image_name_right);
		if (!image.empty())
			cvtColor(image, gray_image, CV_BGR2GRAY);

		bool found = findChessboardCorners(image, board_size, corners_right, CV_CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE | CV_CALIB_CB_FILTER_QUADS);
		if (found) {
			cornerSubPix(gray_image, corners_right, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			drawChessboardCorners(gray_image, board_size, corners_right, found);

			image_points_right.push_back(corners_right);
		}
	}

	calibrateCamera(object_points, image_points_right, image.size(), intrinsic_right, dist_coeffs_right, rvecs_right, tvecs_right);

	std::cout << "focal length right " << intrinsic_right.at<double>(0, 0) << std::endl;
/*
	//REMOVE DISTORTION - TEST
	
	img = imread("right-2.jpg");
	undistort(img, img_undist, intrinsic_right, dist_coeffs_right);
	imshow("dist", img);
	imshow("undist", img_undist);
	waitKey(0);
*/
	//STEREO CALIBRATION
	Mat R = Mat(3, 3, CV_64F);
	Mat T = Mat(3, 1, CV_64F);
	Mat E = Mat(3, 3, CV_64F);
	Mat F = Mat(3, 3, CV_64F);

	stereoCalibrate(object_points, image_points_left, image_points_right, intrinsic_left, dist_coeffs_left, intrinsic_right, dist_coeffs_right, image.size(), R, T, E, F, CV_CALIB_FIX_INTRINSIC, TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 100, 1e-5));

	//STEREO RECTIFICATION
	Mat R1 = Mat(3, 3, CV_64F);
	Mat R2 = Mat(3, 3, CV_64F);
	Mat P1 = Mat(3, 4, CV_64F);
	Mat P2 = Mat(3, 4, CV_64F);
	Mat Q = Mat(4, 4, CV_64F);

	stereoRectify(intrinsic_left, dist_coeffs_left, intrinsic_right, dist_coeffs_right, image.size(), R, T, R1, R2, P1, P2, Q, CV_CALIB_ZERO_DISPARITY);

	std::vector<Point3f> input;
	std::vector<Point3f> output;

	input.push_back(Point3f(394, 284, 113));
	perspectiveTransform(input, output, Q);

	std::cout << output[0] << std::endl;
}	
