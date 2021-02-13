

all:
	g++ Principal.cpp -I/home/jmurillov1/opencv_build/opencv/opencvi/include/opencv4 -L/home/jmurillov1/opencv_build/opencv/opencvi/lib -lopencv_core -lopencv_highgui -lopencv_video -lopencv_imgproc -lopencv_imgcodecs -lopencv_videoio -o vision.bin
	
run:
	./vision.bin