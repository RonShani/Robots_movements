#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include <Arduino.h>
#include <Tachometer.h>
#include "wires_manager.hpp"
#include "driving_constants.hpp"
#include "motors_pwm_calculations.hpp"

class MotorsDriver{
public:
    MotorsDriver() = default;
    MotorsDriver(WiresManager &a_left, WiresManager &a_right);
    void leftFWD();
    void leftBWD();
    void rightFWD();
    void rightBWD();
    void rightSTP();
    void leftSTP();
    WiresManager &left();
    WiresManager &right();
    

private:
    WiresManager &m_left;
    WiresManager &m_right;
};


#endif //MOTOR_DRIVER_HPP