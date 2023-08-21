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
#include <Tachometer.h>
#include <LinearRegression.h>

#include "motors_drivers.hpp"
#include "wires_manager.hpp"
#include "motors_pwm_calculations.hpp"
#include "driving_constants.hpp"

void messageCb( const std_msgs::String& toggle_msg);
void getMaxSpeed();
void publishSpeeds();
void twistcb( const geometry_msgs::Twist& twstmsg);
void check_node_connect();
void cntL();
void cntR();
void motor_running();


uint8_t cctoi(const char cc);
void linearRegressionOfPWMS();
void FloatToMsg(String txt, double number, uint8_t digits);
void printFloat(double number, uint8_t digits);

const int ti=14;


LinearRegression lr = LinearRegression();
PWM_Calculation pwm_calculator;
DrivingConstants driving_constants;

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



const char* ssid = "NETGEAR";
const uint16_t serverPort = 11411;
std_msgs::String str_msg;
geometry_msgs::Twist vl_msg;


void messageCb( const std_msgs::String& toggle_msg)
{
  Serial.println(toggle_msg.data);
  if(toggle_msg.data[0]=='s'){
    getMaxSpeed();
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
  //nh.advertise(speeder);
  analogWriteFreq(40000);
  attachInterrupt(digitalPinToInterrupt(left_motor.encoder()), cntL, FALLING);
  attachInterrupt(digitalPinToInterrupt(right_motor.encoder()), cntR, FALLING);

}

void publishSpeeds()
{
  float mpsLnr=pwm_calculator.getMetersPerSecondLinear();
  if (Vl_goal<0){
    mpsLnr*=(-1);
  }
  FloatToMsg("Linear:",mpsLnr,6);
}

void loop()
{
  motor_running();
  publishSpeeds();
  delay(10);
}

void motor_running(){
  if((Vl_goal>0)&&(Vr_goal>0)){
    rightBWD();
    leftFWD();
    analogWrite(right_motor.speed(), mps_to_pwmRB(Vr_goal));
    analogWrite(left_motor.speed(), mps_to_pwmLF(Vl_goal));
  }
  else if((Vl_goal<0)&&(Vr_goal<0)){
    rightFWD();
    leftBWD();
    analogWrite(right_motor.speed(), mps_to_pwmRF(Vr_goal));
    analogWrite(left_motor.speed(), mps_to_pwmLB(Vl_goal));
  }
  else if((Vl_goal>0)&&(Vr_goal<0)){
    rightFWD();
    leftFWD();
    analogWrite(right_motor.speed(), mps_to_pwmRF(Vr_goal));
    analogWrite(left_motor.speed(), mps_to_pwmLF(Vl_goal));
  }
  else if((Vl_goal<0)&&(Vr_goal>0)){
    rightBWD();
    leftBWD();
    analogWrite(right_motor.speed(), mps_to_pwmRB(Vr_goal));
    analogWrite(left_motor.speed(), mps_to_pwmLB(Vl_goal));
  }
  else{
    leftSTP();
    rightSTP();
  }
}
uint8_t cctoi(const char cc){
  switch(cc) {
  case '0':
    return 0;
    break;
  case '1':
    return 1;
    break;
  case '2':
    return 2;
    break;
  case '3':
    return 3;
    break;
  case '4':
    return 4;
    break;
  case '5':
    return 5;
    break;
  case '6':
    return 6;
    break;
  case '7':
    return 7;
    break;
  case '8':
    return 8;
    break;
  case '9':
    return 9;
    break;
  default:
    return 10;
    break;
  }
}
void check_node_connect(){
  if (nh.connected()){
      str_msg.data = "OK";
      chatter.publish( &str_msg );
    }
  nh.spinOnce();
}

IRAM_ATTR void cntR(){
    tachoR.tick();
}
IRAM_ATTR void cntL() {
    tachoL.tick();
}



void FloatToMsg(String txt, double number, uint8_t digits) 
{ 
String msgTot=txt;


  if (number < 0.0)
  {
     msgTot.concat("-");
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  
  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  msgTot.concat(String(int_part));

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    msgTot.concat("."); 

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    msgTot.concat(String(toPrint));
    remainder -= toPrint; 
  }
  //Serial.println();
  //Serial.println(msgTot);
  if (nh.connected()){
      const char *C = msgTot.c_str();
      str_msg.data = C;
      chatter.publish( &str_msg );
   }
   nh.spinOnce(); 
}
/*
   pwms[i]=getMetersPerSecond((t1-t0),posL);
   lr.learn(pwms[i],pw);
   Serial.print(pw);
   Serial.print(":");
   Serial.println(tmpRPM);
   printFloat(pwms[i],6);
   Serial.println();
   Serial.println(pwms[i]);
  
    Serial.println("End learn");

    Serial.print("Result: 0.12 -> ");
    Serial.println(lr.calculate(0.12));

    Serial.print("Result: 0.01 -> ");
    Serial.println(lr.calculate(0.01));

    Serial.print("Correlation: ");
    Serial.println(lr.correlation());

    Serial.print("Values: ");
    lr.getValues(values);
    Serial.print("Y = ");
    Serial.print(values[0]);
    Serial.print("*X + ");
    Serial.println(values[1]); 
    */
