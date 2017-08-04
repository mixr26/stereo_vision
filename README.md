# stereo_vision
Distance measuring using stereo vision (OpenCV)

Stereo_calib.cpp contains camera calibration and rectification functions. It outputs the Q matrix, which is saved into q_matrix text file.

Stereo_vision.cpp contains distance measuring functions.

1. Compile using make
2. Run stereo_calib to calibrate the cameras usign pictures stored into calib_photos folder.
3. Run stereo_vision to calculate the object distance, by inputing x and y coordinates of the object on the left picture, and also the disparity between the pictures taken by left and right camera respectively as program arguments (object detection coming soon).
