#include "motors_drivers.hpp"
#include "driving_constants.hpp"
#include "motors_pwm_calculations.hpp"
#include "helpers.hpp"

MotorsDriver::MotorsDriver(WiresManager &a_left, WiresManager &a_right)
: m_left(a_left)
, m_right(a_right)
{
}

void MotorsDriver::leftFWD()
{
    digitalWrite(m_left.forth(),0);
    digitalWrite(m_left.back(),0);
}

void MotorsDriver::leftBWD()
{
    digitalWrite(m_left.forth(),1);
    digitalWrite(m_left.back(),1);
}

void MotorsDriver::rightFWD()
{
    digitalWrite(m_right.forth(),0);
    digitalWrite(m_right.back(),0);
}

void MotorsDriver::rightBWD()
{
    digitalWrite(m_right.forth(),1);
    digitalWrite(m_right.back(),1);
}

void MotorsDriver::rightSTP()
{
    digitalWrite(m_right.speed(),HIGH);
    digitalWrite(m_right.forth(),LOW);
    digitalWrite(m_right.back(),LOW);
}

void MotorsDriver::leftSTP()
{
    digitalWrite(m_left.speed(),HIGH);
    digitalWrite(m_left.forth(),LOW);
    digitalWrite(m_left.back(),LOW);
}

WiresManager &MotorsDriver::left()
{
    return m_left;
}

WiresManager &MotorsDriver::right()
{
    return m_right;
}
