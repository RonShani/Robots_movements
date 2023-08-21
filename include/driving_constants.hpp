#ifndef DRIVING_CONSTANTS_HPP
#define DRIVING_CONSTANTS_HPP

class DrivingConstants{

public:
    float maxLinearSpeedFWD=0.12; //meters per second
    float maxLinearSpeedBWD=(-0.12); //meters per second
    float maxAngularSpeedCW=(-0.12); //radians per second z<0
    float maxAngularSpeedCCW=0.12; //radians per second z>0
    float maxLeftWheelFWD=0.12; //meters per second
    float maxLeftWheelBWD=0.12; //meters per second
    float maxRightWheelFWD=0.12; //meters per second
    float maxRightWheelBWD=0.12; //meters per second
    float Vl_goal=0.0;
    float Vr_goal=0.0;
    float V_goal=0.0;
    float W_goal=0.0;
    float _V=0.0;
    float _W=0.0;
};

#endif // DRIVING_CONSTANTS_HPP