#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

void get_q(char* filename, Mat* Q) {
    FILE* file = fopen(filename, "r");
    float q;

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            fscanf(file, "%f", &q);
            Q->at<double>(i, j) = q;
        }
    }

    fclose(file);
}

Point3f calculate_distance(Point3f in, Mat* Q) {
    vector<Point3f> input, output;
    input.push_back(in);

    perspectiveTransform(input, output, *Q);

    return output[0];
}

void convert_colour_spaces(cv::Mat& frame, cv::Mat& hsv_frame, cv::Mat& threshold_frame, cv::Scalar& min, cv::Scalar& max) {
	cv::cvtColor(frame, hsv_frame, CV_BGR2HSV);
	cv::inRange(hsv_frame, min, max, threshold_frame);

	cv::Mat str_el = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	morphologyEx(threshold_frame, threshold_frame, cv::MORPH_OPEN, str_el);
	morphologyEx(threshold_frame, threshold_frame, cv::MORPH_CLOSE, str_el);
}

void find_rectangles(std::vector<std::vector<cv::Point>> contours, std::vector<cv::Vec4i> hierarchy, std::vector<cv::Rect> rectangles, cv::Rect& max_rect, cv::Mat& threshold_frame) {
	int count;

	cv::findContours(threshold_frame.clone(), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

	count = contours.size();
	for (int i = 0; i < count; i++) {
		cv::RotatedRect rrect = cv::minAreaRect(contours[i]);
		cv::Rect rect = rrect.boundingRect();

		rectangles.push_back(rect);
	}

	max_rect = *rectangles.begin();
	for (auto it = rectangles.begin(); it != rectangles.end(); ++it) {
		cv::Rect r = *it;

		if (r.height * r.width > max_rect.height * max_rect.width) {
			max_rect = r;
		}
	}
}

Point3f detect_object(char* left_image, char* right_image) {
    Mat frame_left = imread(left_image);
    Mat frame_right = imread(right_image);
    Mat hsv_frame_left, hsv_frame_right;

    vector<vector<Point>> contours_left, contours_right;
    vector<Vec4i> hierarchy_left, hierarchy_right;
    vector<Rect> rectangles_left, rectangles_right;

    Scalar min(100, 100, 100);
    Scalar max(110, 255, 255);
    Scalar red(255, 0, 0);
    Mat threshold_frame_left, threshold_frame_right;

    Rect max_rect_left, max_rect_right;

    convert_colour_spaces(frame_left, hsv_frame_left, threshold_frame_left, min, max);
    convert_colour_spaces(frame_right, hsv_frame_right, threshold_frame_right, min, max);

    find_rectangles(contours_left, hierarchy_left, rectangles_left, max_rect_left, threshold_frame_left);
    find_rectangles(contours_right, hierarchy_right, rectangles_right, max_rect_right, threshold_frame_right);

    rectangle(threshold_frame_right, max_rect_right, red, 3, 8, 0);
    rectangle(threshold_frame_left, max_rect_left, red, 3, 8, 0);

    imshow("left", threshold_frame_left);
    imshow("right", threshold_frame_right);
    waitKey(0);

    return Point3f(max_rect_left.x, max_rect_left.y, abs(max_rect_left.x - max_rect_right.x));
}

int main(int argc, char** argv) {
    Mat Q = Mat(4, 4, CV_64F);
    Point3f input;

    if (argc == 4) {
        float x = atof(argv[1]);
        float y = atof(argv[2]);
        float disp = atof(argv[3]);
        input = Point3f(x, y, disp);
    } else if (argc == 3) {
        input = detect_object(argv[1], argv[2]);
    }

    get_q((char*)"q_matrix", &Q);

    //Point3f dim = calculate_distance(394, 284, 113, &Q);
    Point3f dim = calculate_distance(input, &Q);
    cout << dim << endl;
}
