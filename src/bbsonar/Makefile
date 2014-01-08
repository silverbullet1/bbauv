LDFLAGS = -L./lib -L/usr/local/lib			
LDLIBS = -lbvtsdk -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_legacy	
CFLAGS = -I./include -I/usr/local/include/opencv -I/usr/local/include/opencv2
GFLAGS = -Wall -g
CC = g++

SOURCE = main.cpp BBSonar.cpp BBSonar.h
OBJ = main.o BBSonar.o

all: main

main: $(OBJ)
	$(CC) $(GFLAGS) -o bbsonar $(OBJ) $(LDFLAGS) $(LDLIBS) 
	
main.o: main.cpp 
	$(CC) $(GFLAGS) -c main.cpp $(CFLAGS)
	
BBSonar.o: BBSonar.cpp BBSonar.h
	$(CC) $(GFLAGS) -c BBSonar.cpp $(CFLAGS)

.PHONY:
	all clean
	
clean:
	rm bbsonar $(OBJ) *.pgm *.ppm *.son *~
	
