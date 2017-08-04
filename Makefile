all:
	g++ stereo_calib.cpp -o stereo_calib -g -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc -lopencv_calib3d
	g++ stereo_vision.cpp -o stereo_vision -g -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc -lopencv_calib3d

clean:
	rm stereo_calib stereo_vision
