all:
	g++ src/* -o stereo_vision -g -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc -lopencv_calib3d

clean:
	rm stereo_vision
