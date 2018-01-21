# stereo_vision
Distance measuring using stereo vision (OpenCV)

StereoCalib class provides an interface for calibrating the cameras. Calibration photos (checkerboard) are provided in the calib_photos folder. They must be listed in additional_files/left.txt (pictures taken by the left camera) and additional_files/right.txt (pictures taken by the right camera) text files, which are loaded by the class.

StereoVision class provides an interface for detecting an object of a given colour, and transforming its 2D image coordinates into 3D real-world coordinates.
