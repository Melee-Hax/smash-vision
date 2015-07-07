PLATFORM = $(shell uname)

CFLAGS=-L/usr/local/lib \-I/usr/local/include
## Mac OS X
LDFLAGS+= -lopencv_core -lopencv_calib3d -lopencv_features2d -lopencv_flann -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc -lopencv_ml -lopencv_objdetect -lopencv_photo -lopencv_shape -lopencv_stitching -lopencv_superres -lopencv_ts -lopencv_video -lopencv_videoio -lopencv_videostab

CC = g++ -O3 -w

PROGS = main

default: $(PROGS)

main: main.o ObjectTracker.o HsvHistogram.o LineHoughTransform.o TrackingParticleFilter.o
	$(CC) $(CFLAGS) -o $@ main.o ObjectTracker.o HsvHistogram.o LineHoughTransform.o TrackingParticleFilter.o $(LDFLAGS)

main.o: main.cpp  ObjectTracker.h ObjectTracker.h
	$(CC) $(CFLAGS) -c main.cpp  -o $@

ObjectTracker.o: ObjectTracker.cpp ObjectTracker.h 
	$(CC) $(CFLAGS) -c ObjectTracker.cpp -o $@

HsvHistogram.o: HsvHistogram.h HsvHistogram.cpp
	$(CC) $(CFLAGS) -c HsvHistogram.cpp -o $@

LineHoughTransform.o: LineHoughTransform.cpp LineHoughTransform.h
	$(CC) $(CFLAGS) -c LineHoughTransform.cpp -o $@

TrackingParticleFilter.o: TrackingParticleFilter.cpp TrackingParticleFilter.h
	$(CC) $(CFLAGS) -c TrackingParticleFilter.cpp -o $@

clean::	
	rm *.o
	rm main


