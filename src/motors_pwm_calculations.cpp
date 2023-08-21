#include "motors_pwm_calculations.hpp"

#include <Arduino.h>
#include <geometry_msgs/Twist.h>
#include <LinearRegression.h>

#include "helpers.hpp"
#include "motors_drivers.hpp"

PWM_Calculation::PWM_Calculation()
: tachoL{}
, tachoR{}
{
}

float PWM_Calculation::getOmega(float _vl, float _vr)
{
    return 0.0f;
}

float PWM_Calculation::getOmega(float _vl, float _vr){
  return (((_vr-_vl)/L)*R);
}
float PWM_Calculation::getLinear(float _vl, float _vr){
  return (((_vr+_vl)/2)*R);
}
float PWM_Calculation::getVr(float Vx, float W){
  return (((2*Vx)+(W*L))/(2*R));
}
float PWM_Calculation::getVl(float Vx, float W){
  return (((2*Vx)-(W*L))/(2*R));
}


float PWM_Calculation::getMetersPerSecondLinear(){
  return (Pi_Rsqr_div_tiks_full_round*(tachoL.getHz()+tachoR.getHz()));
}
float PWM_Calculation::getMetersPerSecondAngular(){
  return (omegaMultiplier*(tachoR.getHz()-tachoL.getHz()));
}


uint8_t PWM_Calculation::mps_to_pwmLF(float mps){
  return (valuesLF[0]*mps+valuesLF[1]);
}
uint8_t PWM_Calculation::mps_to_pwmRF(float mps){
  return (valuesRF[0]*mps+valuesRF[1]);
}
uint8_t PWM_Calculation::mps_to_pwmLB(float mps){
  return(valuesLB[0]*mps+valuesLB[1]);
}
uint8_t PWM_Calculation::mps_to_pwmRB(float mps){
  return(valuesRB[0]*mps+valuesRB[1]);
}

void PWM_Calculation::twistcb(const geometry_msgs::Twist &twstmsg, DrivingConstants &a_consts)
{
  if(twstmsg.linear.x>0.0){
    if(twstmsg.linear.x>a_consts.maxLinearSpeedFWD){
      Vl_goal=getVl(a_consts.maxLinearSpeedFWD,twstmsg.angular.z);
      Vr_goal=getVr(a_consts.maxLinearSpeedFWD,twstmsg.angular.z);
    }
  }
  else if(twstmsg.linear.x<0.0){
    if(twstmsg.linear.x<a_consts.maxLinearSpeedBWD){
      Vl_goal=getVl(a_consts.maxLinearSpeedBWD,twstmsg.angular.z);
      Vr_goal=getVr(a_consts.maxLinearSpeedBWD,twstmsg.angular.z);
    }
  }
  else{
        Vl_goal=getVl(twstmsg.linear.x,twstmsg.angular.z);
        Vr_goal=getVr(twstmsg.linear.x,twstmsg.angular.z);      
    }
}

void PWM_Calculation::linearRegressionOfPWMS(LinearRegression &a_linearRegression, Helpers &a_helpers)
{

  for(int i=0;i<VARIOUS_DRIVING_SPEEDS;i++){
   int pw=i*15;
   a_linearRegression.learn(pwmsRF[i],pw);
   Serial.print(pw);
   Serial.print(" RF:");
   Serial.println(pwmsRF[i]);
   a_helpers.printFloat(pwmsRF[i],6);
   Serial.println();
  }
    Serial.print("Result: 0.12 -> ");
    a_helpers.printFloat(a_linearRegression.calculate(0.12),6);

    Serial.print("Result: 0.01 -> ");
    a_helpers.printFloat(a_linearRegression.calculate(0.01),6);

    Serial.print("Correlation: ");
    Serial.println(a_linearRegression.correlation());

    Serial.print("Values: ");
    a_linearRegression.parameters(valuesRF);
    Serial.print("Y = ");
    Serial.print(valuesRF[0]);
    Serial.print("*X + ");
    Serial.println(valuesRF[1]);
    a_linearRegression.reset();

  for(int i=0;i<VARIOUS_DRIVING_SPEEDS;i++){
   int pw=i*15;
   a_linearRegression.learn(pwmsRB[i],pw);
   Serial.print(pw);
   Serial.print(" RB:");
   Serial.println(pwmsRB[i]);
   a_helpers.printFloat(pwmsRB[i],6);
   Serial.println();
  }
    Serial.println("End learn");

    Serial.print("Result: 0.12 -> ");
    a_helpers.printFloat(a_linearRegression.calculate(0.12),6);

    Serial.print("Result: 0.01 -> ");
    a_helpers.printFloat(a_linearRegression.calculate(0.01),6);

    Serial.print("Correlation: ");
    Serial.println(a_linearRegression.correlation());

    Serial.print("Values: ");
    a_linearRegression.parameters(valuesRB);
    Serial.print("Y = ");
    Serial.print(valuesRB[0]);
    Serial.print("*X + ");
    Serial.println(valuesRB[1]);
    a_linearRegression.reset();

 //------------------------------------------//
   
  for(int i=0;i<VARIOUS_DRIVING_SPEEDS;i++){
   int pw=i*15;
   a_linearRegression.learn(pwmsLF[i],pw);
   Serial.print(pw);
   Serial.print(" LF:");
   Serial.println(pwmsLF[i]);
   a_helpers.printFloat(pwmsLF[i],6);
   Serial.println();
  }
    Serial.println("End learn");

    Serial.print("Result: 0.12 -> ");
    a_helpers.printFloat(a_linearRegression.calculate(0.12),6);

    Serial.print("Result: 0.01 -> ");
    a_helpers.printFloat(a_linearRegression.calculate(0.01),6);

    Serial.print("Correlation: ");
    Serial.println(a_linearRegression.correlation());

    Serial.print("Values: ");
    a_linearRegression.parameters(valuesLF);
    Serial.print("Y = ");
    Serial.print(valuesLF[0]);
    Serial.print("*X + ");
    Serial.println(valuesLF[1]);
    a_linearRegression.reset();

  for(int i=0;i<VARIOUS_DRIVING_SPEEDS;i++){
   int pw=i*15;
   a_linearRegression.learn(pwmsLB[i],pw);
   Serial.print(pw);
   Serial.print(" LB:");
   Serial.println(pwmsLB[i]);
   a_helpers.printFloat(pwmsLB[i],6);
   Serial.println();
  }
    Serial.println("End learn");

    Serial.print("Result: 0.12 -> ");
    a_helpers.printFloat(a_linearRegression.calculate(0.12),6);

    Serial.print("Result: 0.01 -> ");
    a_helpers.printFloat(a_linearRegression.calculate(0.01),6);

    Serial.print("Correlation: ");
    Serial.println(a_linearRegression.correlation());

    Serial.print("Values: ");
    a_linearRegression.parameters(valuesLB);
    Serial.print("Y = ");
    Serial.print(valuesLB[0]);
    Serial.print("*X + ");
    Serial.println(valuesLB[1]);
    a_linearRegression.reset();
}

void PWM_Calculation::getMaxSpeed(LinearRegression &a_linearRegression,DrivingConstants &a_constants, MotorsDriver &a_driver, Helpers &a_helpers)
{ 
    Helpers print_helpers;
  Serial.println();
  Serial.println("Getting max speeds");
  a_driver.rightBWD();
  a_driver.leftFWD();
  analogWrite(a_driver.right().speed(),0);
  analogWrite(a_driver.left().speed(),0);
  delayMicroseconds(1000000);
  yield();
  a_constants.maxLeftWheelFWD=(tachoL.getHz()/tiks_full_round)*wheel_scope;
  a_constants.maxRightWheelBWD=(tachoR.getHz()/tiks_full_round)*wheel_scope;
  a_driver.leftSTP();
  a_driver.rightSTP();
  if(a_constants.maxLeftWheelFWD > a_constants.maxRightWheelBWD){a_constants.maxLinearSpeedFWD=a_constants.maxRightWheelBWD;}
  else{a_constants.maxLinearSpeedFWD=a_constants.maxLeftWheelFWD;}
  delayMicroseconds(1000000);
  yield();
  a_driver.rightFWD();
  a_driver.leftBWD();
  analogWrite(a_driver.right().speed(),0);
  analogWrite(a_driver.left().speed(),0);
  delayMicroseconds(1000000);
  yield();
  a_constants.maxLeftWheelBWD=(tachoL.getHz()/tiks_full_round)*wheel_scope;
  a_constants.maxRightWheelFWD=(tachoR.getHz()/tiks_full_round)*wheel_scope;
  a_driver.leftSTP();
  a_driver.rightSTP();
  if(a_constants.maxLeftWheelBWD > a_constants.maxRightWheelFWD){a_constants.maxLinearSpeedBWD=a_constants.maxRightWheelFWD*(-1);}
  else{a_constants.maxLinearSpeedBWD=a_constants.maxLeftWheelBWD*(-1);}
  delayMicroseconds(1000000);
  yield();
  Serial.print("maxLeftWheelFWD:");
  Serial.println(a_constants.maxLeftWheelFWD);
  Serial.print("maxLeftWheelBWD:");
  Serial.println(a_constants.maxLeftWheelBWD);
  Serial.print("maxRightWheelFWD:");
  Serial.println(a_constants.maxRightWheelFWD);
  Serial.print("maxRightWheelBWD:");
  Serial.println(a_constants.maxRightWheelBWD);
  
  Serial.print("maxLinearSpeedBWD:");
  Serial.println(a_constants.maxLinearSpeedBWD);
  Serial.print("maxLinearSpeedFWD:");
  Serial.println(a_constants.maxLinearSpeedFWD);
  a_constants.maxAngularSpeedCW = ((a_constants.maxRightWheelBWD - a_constants.maxLeftWheelFWD)*R)/L;  
  a_constants.maxAngularSpeedCCW = ((a_constants.maxRightWheelFWD - a_constants.maxLeftWheelBWD)*R)/L;
  
  Serial.print("maxAngularSpeedCW:");
  Serial.println(a_constants.maxAngularSpeedCW);
  Serial.print("maxAngularSpeedCCW:");
  Serial.println(a_constants.maxAngularSpeedCCW);
  
  unsigned long t0=0;
  int pw=0;
  
  for(int i=0;i<VARIOUS_DRIVING_SPEEDS;i++){
   a_driver.rightBWD();
   a_driver.leftFWD();
   pw=i*15;
   analogWrite(a_driver.right().speed(),pw);
   analogWrite(a_driver.left().speed(),pw);
   delayMicroseconds(50000);
   yield();
   t0=millis();
   while(t0>(millis()-5000)){
    delayMicroseconds(5000);
    yield();
    pwmsLF[i]=tachoL.getHz()/tiks_full_round;
    pwmsRB[i]=tachoR.getHz()/tiks_full_round;
    if((pwmsLF[i]*((millis()-t0)/1000)>1)||(pwmsRB[i]*((millis()-t0)/1000)>1)){break;}
   }
    pwmsLF[i]*=wheel_scope;
    pwmsRB[i]*=wheel_scope;
   Serial.println();
   Serial.print(pw);
   Serial.print(" LF:");
   print_helpers.printFloat(pwmsLF[i],6);
   Serial.println();
   Serial.print(pw);
   Serial.print(" RB:");
   print_helpers.printFloat(pwmsRB[i],6);
   a_driver.leftSTP();
   a_driver.rightSTP();
   delayMicroseconds(1000000);
   yield();
   Serial.println();
   
   a_driver.rightFWD();
   a_driver.leftBWD();
   analogWrite(a_driver.right().speed(),pw);
   analogWrite(a_driver.left().speed(),pw);
   delayMicroseconds(50000);
   yield();
   t0=millis();
   
   while(t0>(millis()-5000)){
    delayMicroseconds(5000);
    yield();
    pwmsLB[i]=tachoL.getHz()/tiks_full_round;
    pwmsRF[i]=tachoR.getHz()/tiks_full_round;
    if((pwmsLB[i]*((millis()-t0)/1000)>1)||(pwmsRF[i]*((millis()-t0)/1000)>1)){break;}
   }
    pwmsLB[i]*=wheel_scope;
    pwmsRF[i]*=wheel_scope;
   Serial.println();
   Serial.print(pw);
   Serial.print(" LB:");
   print_helpers.printFloat(pwmsLB[i],6);
   Serial.println();
   Serial.print(pw);
   Serial.print(" RF:");
   print_helpers.printFloat(pwmsRF[i],6);
   a_driver.leftSTP();
   a_driver.rightSTP();
   delayMicroseconds(1000000);
   yield();
   Serial.println();
   }
   
   linearRegressionOfPWMS(a_linearRegression, a_helpers);
}