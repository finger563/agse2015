# Include OpenCV library and build main.cpp
all: uip.cpp
	g++ -o uip uip.cpp `pkg-config --cflags --libs opencv`

clean: 
	$(RM) uip