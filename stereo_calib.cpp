#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

//square size in meters
#define SQUARE_SIZE 0.0235

std::vector<Point3f> obj;
Size image_size;

void calibrate_camera(FILE* file, vector<vector<Point3f>>* object_points, vector<vector<Point2f>>* image_points, vector<Point2f>* corners, Mat* intrinsic, Mat* dist_coeffs, vector<Mat>* rvecs, vector<Mat>* tvecs, Size board_size) {
    Mat image, gray_image;
    char image_name[40];

    object_points->clear();

    while (fscanf(file, "%s", image_name) != EOF) {
		image = imread(image_name);
		if (!image.empty())
			cvtColor(image, gray_image, CV_BGR2GRAY);

		bool found = findChessboardCorners(image, board_size, *corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
		if (found) {
			cornerSubPix(gray_image, *corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			drawChessboardCorners(gray_image, board_size, *corners, found);

            image_size = image.size();
			image_points->push_back(*corners);
			object_points->push_back(obj);
		}
	}

	calibrateCamera(*object_points, *image_points, image.size(), *intrinsic, *dist_coeffs, *rvecs, *tvecs);

	cout << "focal length: " << intrinsic->at<double>(0, 0) << endl;
}

void remove_distortion(FILE* file, Mat* intrinsic, Mat* dist_coeffs) {
    Mat dist_img, undist_img;
    char image_name[40];

    rewind(file);

    fscanf(file, "%s", image_name);
    cout << image_name << endl;
    dist_img = imread(image_name);

    undistort(dist_img, undist_img, *intrinsic, *dist_coeffs);

    imshow("dist", dist_img);
    imshow("undist", undist_img);
    waitKey(0);
}

void output_q(char* filename, Mat Q) {
    FILE* q_output = fopen(filename, "w");

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            fprintf(q_output, "%f ", Q.at<double>(i, j));
        }
        fprintf(q_output, "\n");
    }

    fclose(q_output);
}

int main(int argc, char** argv) {
	int num_boards = 14;
	int num_corners_hor = 9;
	int num_corners_ver = 7;
	int num_squares = num_corners_ver * num_corners_hor;
	Size board_size = Size(num_corners_hor, num_corners_ver);

	char image_name_left[40];
	char image_name_right[40];
	char* filename_left = argv[1];
	char* filename_right = argv[2];
	FILE* file_left = fopen("left.txt", "r");
	FILE* file_right = fopen("right.txt", "r");

	vector<vector<Point3f>> object_points;
	vector<vector<Point2f>> image_points_left;
	vector<vector<Point2f>> image_points_right;
	vector<Point2f> corners_left;
	vector<Point2f> corners_right;

	Mat image;
	Mat gray_image;
	Mat intrinsic_left = Mat(3, 3, CV_64F);
	Mat intrinsic_right = Mat(3, 3, CV_64F);
	Mat dist_coeffs_left;
	Mat dist_coeffs_right;
	vector<Mat> rvecs_left;
	vector<Mat> tvecs_left;
	vector<Mat> rvecs_right;
	vector<Mat> tvecs_right;

	intrinsic_left.ptr<float>(0)[0] = 1;
	intrinsic_right.ptr<float>(0)[0] = 1;

	for (int i = 0; i < num_corners_ver; i++) {
		for (int j = 0; j < num_corners_hor; j++) {
			obj.push_back(Point3f(float(j * SQUARE_SIZE), float(i * SQUARE_SIZE), 0.0f));
		}
	}

    calibrate_camera(file_left, &object_points, &image_points_left, &corners_left, &intrinsic_left, &dist_coeffs_left, &rvecs_left, &tvecs_left, board_size);
//  remove_distortion(file_left, &intrinsic_left, &dist_coeffs_left);

    calibrate_camera(file_right, &object_points, &image_points_right, &corners_right, &intrinsic_right, &dist_coeffs_right, &rvecs_right, &tvecs_right, board_size);

	//STEREO CALIBRATION
	Mat R = Mat(3, 3, CV_64F);
	Mat T = Mat(3, 1, CV_64F);
	Mat E = Mat(3, 3, CV_64F);
	Mat F = Mat(3, 3, CV_64F);

	stereoCalibrate(object_points, image_points_left, image_points_right, intrinsic_left, dist_coeffs_left, intrinsic_right, dist_coeffs_right, image_size, R, T, E, F, CV_CALIB_FIX_INTRINSIC, TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 100, 1e-5));

	//STEREO RECTIFICATION
	Mat R1 = Mat(3, 3, CV_64F);
	Mat R2 = Mat(3, 3, CV_64F);
	Mat P1 = Mat(3, 4, CV_64F);
	Mat P2 = Mat(3, 4, CV_64F);
	Mat Q = Mat(4, 4, CV_64F);

	stereoRectify(intrinsic_left, dist_coeffs_left, intrinsic_right, dist_coeffs_right, image_size, R, T, R1, R2, P1, P2, Q, CV_CALIB_ZERO_DISPARITY);

    output_q((char*)"q_matrix", Q);

    fclose(file_left);
    fclose(file_right);
}
