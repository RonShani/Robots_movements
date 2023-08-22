#ifndef HELPERS_HPP
#define HELPERS_HPP
#include <std_msgs/UInt32.h>
#include <string>
#include <std_msgs/UInt8.h>
#include <stdlib.h>

class Helpers{
public:
    Helpers() = default;
    ~Helpers() = default;
    
    void printFloat(double number, uint8_t digits);
    uint8_t cctoi(const char cc);
    std::string FloatToMsg(std::string txt, double number, uint8_t digits);

};

#endif // HELPERS_HPP