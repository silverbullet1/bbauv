#include <serial/serial.h>
#include <fstream>
#include <iostream>
#include <unistd.h>


int main(int argc, char **argv)
{
    if(argc < 2){
        std::cerr << "Not enough arguments";
        return(1);
    }

    serial::Serial loopback("/dev/tty.usbserial-A602H7YO", 9600,
                            serial::Timeout::simpleTimeout(250));
    std::ifstream capture(argv[1],
                          std::ios::binary);
    loopback.setFlowcontrol(serial::flowcontrol_none);

    while(capture.good()){
        uint8_t c = capture.get();
        loopback.write(&c, sizeof(c));
        if(capture.eof()){
            capture.clear();
            capture.seekg(0, std::ios::beg);
        }
        usleep(100);
    }

    return 0;
}
