all:
	g++ src/* -o stereo_vision -g -I/usr/local/include/opencv4/ -L/usr/local/lib/ -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc -lopencv_calib3d

clean:
	rm stereo_vision
