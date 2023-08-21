#ifndef PWM_CALCULATORS_HPP
#define PWM_CALCULATORS_HPP

#include <Arduino.h>
#include <Tachometer.h>
#include <geometry_msgs/Twist.h>
#include <LinearRegression.h>

#include "driving_constants.hpp"
#include "helpers.hpp"
#include "motors_drivers.hpp"

#define VARIOUS_DRIVING_SPEEDS 14

class PWM_Calculation{
public:
    PWM_Calculation();
    float getOmega(float _vl, float _vr);
    float getLinear(float _vl, float _vr);
    float getVr(float Vx, float W);
    float getVl(float Vx, float W);
    float getMetersPerSecondLinear();
    float getMetersPerSecondAngular();
    uint8_t mps_to_pwmLF(float mps);
    uint8_t mps_to_pwmRF(float mps);
    uint8_t mps_to_pwmLB(float mps);
    uint8_t mps_to_pwmRB(float mps);
    void twistcb( const geometry_msgs::Twist& twstmsg, DrivingConstants &a_consts);
    void linearRegressionOfPWMS(LinearRegression &a_linearRegression, Helpers &a_helpers);
    void getMaxSpeed(LinearRegression &a_linearRegression, DrivingConstants &a_constants, MotorsDriver &a_driver, Helpers &a_helpers);

public:
    Tachometer tachoL;
    Tachometer tachoR;
    const float L = 0.48; //distance between wheels
    const float R = 0.095; //wheel radius meters
    const float tiks_full_round = 1200.00; //this number is observatory
    const float Pi_Rsqr_div_tiks_full_round=(PI*R*R)/tiks_full_round;
    const float omegaMultiplier = (2*PI*R*R)/(tiks_full_round/L);
    const float wheel_scope = 0.29845; // meters
    double valuesLF[2];
    double valuesLB[2];
    double valuesRF[2];
    double valuesRB[2];
    float pwmsLF[VARIOUS_DRIVING_SPEEDS];
    float pwmsLB[VARIOUS_DRIVING_SPEEDS];
    float pwmsRF[VARIOUS_DRIVING_SPEEDS];
    float pwmsRB[VARIOUS_DRIVING_SPEEDS];
    float Vl_goal=0.0;
    float Vr_goal=0.0;
    float V_goal=0.0;
    float W_goal=0.0;
    float _V=0.0;
    float _W=0.0;
};


#endif //PWM_CALCULATORS_HPP