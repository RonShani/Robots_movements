#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include <Arduino.h>
#include <Tachometer.h>
#include "wires_manager.hpp"
#include "driving_constants.hpp"

class PWM_Calculation;

class MotorsDriver{

public:
    MotorsDriver(WiresManager &a_left, WiresManager &a_right);
    ~MotorsDriver() = default;
    
    void leftFWD();
    void leftBWD();
    void rightFWD();
    void rightBWD();
    void rightSTP();
    void leftSTP();
    WiresManager &left();
    WiresManager &right();
    void motor_running(float a_velocity_left_goal, float a_velocity_right_goal, PWM_Calculation &a_pwm_calc);

private:
    WiresManager &m_left;
    WiresManager &m_right;
};


#endif //MOTOR_DRIVER_HPP