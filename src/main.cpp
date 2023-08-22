/*
 * Converting standart ros geometric messages
 * to movement in real world
 * for a 4 motors robot controlled with 2 motor-controllers
 * 
 * 
 * This code was made for LivePharm */
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <Tachometer.h>
#include <LinearRegression.h>

#include "motors_drivers.hpp"
#include "wires_manager.hpp"
#include "motors_pwm_calculations.hpp"
#include "driving_constants.hpp"

void messageCb( const std_msgs::String& toggle_msg);
void check_node_connect();
void twistcb(const geometry_msgs::Twist &twstmsg);

LinearRegression lr = LinearRegression();
PWM_Calculation pwm_calculator;
DrivingConstants driving_constants;
Helpers helpers;

/*Left wheels
pin 16 = ccw; //D0 white
pin 13 = cw; //D7 white
pin 4 = speed; //D2 blue
pin 14 = encoder; //D5 yellow */
WiresManager left_motor{16, 13, 4, 14};

/*Right wheels
pin 5 = ccw; //D0 white
pin 12 = cw; //D7 white
pin 0 = speed; //D2 blue
pin 2 = encoder; //D5 yellow */
WiresManager right_motor{5, 12, 0, 2};

MotorsDriver motors_driver{left_motor, right_motor};

const char* ssid = "NETGEAR";
const uint16_t serverPort = 11411;
std_msgs::String str_msg;
geometry_msgs::Twist vl_msg;


void messageCb( const std_msgs::String& toggle_msg)
{
  Serial.println(toggle_msg.data);
  if(toggle_msg.data[0]=='s'){
    pwm_calculator.getMaxSpeed(lr, driving_constants, motors_driver, helpers);
  }
}
IPAddress server(10,0,0,45);
ros::NodeHandle nh;
ros::Publisher chatter("chatter", &str_msg);
ros::Subscriber<geometry_msgs::Twist> twistmcd("/cmd_vel", twistcb);
ros::Publisher velpub("/vel_mux", &vl_msg);
ros::Subscriber<std_msgs::String> sub("/debugme", messageCb );



 
void setup()
{
  Serial.begin(115200);
  WiFi.begin(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(twistmcd);
  nh.advertise(chatter);
  
  analogWriteFreq(40000);
  attachInterrupt(digitalPinToInterrupt(left_motor.encoder()), pwm_calculator.cntL, FALLING);
  attachInterrupt(digitalPinToInterrupt(right_motor.encoder()), pwm_calculator.cntR, FALLING);

}

void publishSpeeds()
{
  float mpsLnr=pwm_calculator.getMetersPerSecondLinear();
  if (driving_constants.Vl_goal<0){
    mpsLnr*=(-1);
  }
  if (nh.connected()){
    std::string speeds_msg = helpers.FloatToMsg("Linear:",mpsLnr,6);
    const char *C = speeds_msg.c_str();
    str_msg.data = C;
    chatter.publish( &str_msg );
  }
  nh.spinOnce(); 
}

void loop()
{
  motors_driver.motor_running(driving_constants.Vl_goal, driving_constants.Vr_goal, pwm_calculator);
  publishSpeeds();
  delay(10);
}


void check_node_connect(){
  if (nh.connected()){
      str_msg.data = "OK";
      chatter.publish( &str_msg );
    }
  nh.spinOnce();
}


void twistcb(const geometry_msgs::Twist &twstmsg)
{
  if(twstmsg.linear.x>0.0){
    if(twstmsg.linear.x>driving_constants.maxLinearSpeedFWD){
      pwm_calculator.Vl_goal=pwm_calculator.getVl(driving_constants.maxLinearSpeedFWD,twstmsg.angular.z);
      pwm_calculator.Vr_goal=pwm_calculator.getVr(driving_constants.maxLinearSpeedFWD,twstmsg.angular.z);
    }
  }
  else if(twstmsg.linear.x<0.0){
    if(twstmsg.linear.x<driving_constants.maxLinearSpeedBWD){
      pwm_calculator.Vl_goal=pwm_calculator.getVl(driving_constants.maxLinearSpeedBWD,twstmsg.angular.z);
      pwm_calculator.Vr_goal=pwm_calculator.getVr(driving_constants.maxLinearSpeedBWD,twstmsg.angular.z);
    }
  }
  else{
        pwm_calculator.Vl_goal=pwm_calculator.getVl(twstmsg.linear.x,twstmsg.angular.z);
        pwm_calculator.Vr_goal=pwm_calculator.getVr(twstmsg.linear.x,twstmsg.angular.z);      
    }
}