#include <iostream>
#include <stdio.h>
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

Point3f calculate_distance(float x, float y, float disparity, Mat* Q) {
    vector<Point3f> input, output;
    input.push_back(Point3f(x, y, disparity));

    perspectiveTransform(input, output, *Q);

    return output[0];
}

int main(int argc, char** argv) {
    Mat Q = Mat(4, 4, CV_64F);

    get_q((char*)"q_matrix", &Q);

    Point3f dim = calculate_distance(394, 284, 113, &Q);
    cout << dim << endl;
}
