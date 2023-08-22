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

void MotorsDriver::motor_running(float a_velocity_left_goal, float a_velocity_right_goal, PWM_Calculation &a_pwm_calc)
{
  if((a_velocity_left_goal>0)&&(a_velocity_right_goal>0)){
    rightBWD();
    leftFWD();
    analogWrite(m_right.speed(), a_pwm_calc.mps_to_pwmRB(a_velocity_right_goal));
    analogWrite(m_left.speed(), a_pwm_calc.mps_to_pwmLF(a_velocity_left_goal));
  }
  else if((a_velocity_left_goal<0)&&(a_velocity_right_goal<0)){
    rightFWD();
    leftBWD();
    analogWrite(m_right.speed(), a_pwm_calc.mps_to_pwmRF(a_velocity_right_goal));
    analogWrite(m_left.speed(), a_pwm_calc.mps_to_pwmLB(a_velocity_left_goal));
  }
  else if((a_velocity_left_goal>0)&&(a_velocity_right_goal<0)){
    rightFWD();
    leftFWD();
    analogWrite(m_right.speed(), a_pwm_calc.mps_to_pwmRF(a_velocity_right_goal));
    analogWrite(m_left.speed(), a_pwm_calc.mps_to_pwmLF(a_velocity_left_goal));
  }
  else if((a_velocity_left_goal<0)&&(a_velocity_right_goal>0)){
    rightBWD();
    leftBWD();
    analogWrite(m_right.speed(), a_pwm_calc.mps_to_pwmRB(a_velocity_right_goal));
    analogWrite(m_left.speed(), a_pwm_calc.mps_to_pwmLB(a_velocity_left_goal));
  }
  else{
    leftSTP();
    rightSTP();
  }
}
