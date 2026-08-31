DESTDIR = /usr/local
MODE = 755
CC = gcc
CXX = g++
CFLAGS = -std=c11 -Wall -Wextra -Wpedantic -O2 $(shell pkg-config --cflags sdl2 libavcodec libavutil libswscale opencv4)
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 $(shell pkg-config --cflags sdl2 libavcodec libavutil libswscale opencv4)
LIBS = $(shell pkg-config --libs sdl2 libavcodec libavutil libswscale opencv4) -lm -lpthread

all: tello

aruco.o: aruco.cpp aruco.h
	$(CXX) $(CXXFLAGS) -c aruco.cpp -o $@

tello.o: tello_ui.c tello.h aruco.h
	$(CC) $(CFLAGS) -c tello_ui.c -o $@

tello_lib.o: tello.c tello.h
	$(CC) $(CFLAGS) -c tello.c -o $@

tello: tello.o tello_lib.o aruco.o
	$(CXX) tello.o tello_lib.o aruco.o -o $@ $(LIBS)

clean:
	rm -f tello tello.o tello_lib.o aruco.o

install: all
	install -d ${DESTDIR}/bin
	install -m ${MODE} tello ${DESTDIR}/bin

uninstall:
	rm -f ${DESTDIR}/bin/tello

.PHONY: all clean install uninstall

