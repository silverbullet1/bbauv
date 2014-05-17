LDFLAGS = -L./lib -L/usr/local/lib
LDLIBS = -lbvtsdk -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_legacy	
CFLAGS = -I./include -I/usr/local/include/opencv -I/usr/local/include  
GFLAGS = -Wall -g
CC = g++

OBJ = main.o Utility.o

all: main

main: $(OBJ)
	$(CC) $(GFLAGS) -o bbsonar $(OBJ) $(LDFLAGS) $(LDLIBS)
	
main.o: main.cpp 
	$(CC) $(GFLAGS) -c main.cpp $(CFLAGS)
	
	
Utility.o: Utility.cpp Utility.h Config.h
	$(CC) $(GFLAGS) -c Utility.cpp $(CFLAGS) 

.PHONY:
	all clean
	
clean:
	rm bbsonar $(OBJ) *.pgm *.ppm *.png *.jpg *.son *.txt *.yml *~
	

