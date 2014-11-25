# Include OpenCV library and build main.cpp
all: main.cpp
	g++ -o canny_edge_detector main.cpp Canny_Edge_Detector.cpp `pkg-config --cflags --libs opencv`

clean: 
	$(RM) canny_edge_detector

