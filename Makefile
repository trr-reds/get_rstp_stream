

all:
	g++ -I. main.cpp -o main -lpthread `pkg-config --cflags --libs gstreamer-1.0` 
