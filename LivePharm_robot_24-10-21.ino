/*
 * Converting standart ros geometric messages
 * to movement in real world
 * for a 4 motors robot controlled with 2 motor-controllers
 * 
 * 
 * This code was made for LivePharm
 */

#include <ESP8266WiFi.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <LinearRegression.h>

const int ti=14;
LinearRegression lr = LinearRegression();
double valuesLF[2];
double valuesLB[2];
double valuesRF[2];
double valuesRB[2];
#include <Tachometer.h>
Tachometer tachoL;
Tachometer tachoR;

const float L = 0.48; //distance between wheels
const float wheel_scope = 0.29845; // meters 
const float R = 0.095; //wheel radius meters
const float tiks_full_round = 1200.00; //this number is observatory
const float Pi_Rsqr_div_tiks_full_round=(PI*R*R)/tiks_full_round;
const float omegaMultiplier = (2*PI*R*R)/(tiks_full_round/L);
float maxLinearSpeedFWD=0.12; //meters per second
float maxLinearSpeedBWD=(-0.12); //meters per second
float maxAngularSpeedCW=(-0.12); //radians per second z<0
float maxAngularSpeedCCW=0.12; //radians per second z>0
float maxLeftWheelFWD=0.12; //meters per second
float maxLeftWheelBWD=0.12; //meters per second
float maxRightWheelFWD=0.12; //meters per second
float maxRightWheelBWD=0.12; //meters per second
void printFloat(double number, uint8_t digits);


//Left wheels
uint8_t DIRLB = 16; //D0 white
uint8_t DIRLF = 13; //D7 white
uint8_t SPDL = 4; //D2 blue
uint8_t ENCL = 14; //D5 yellow

//Right wheels
uint8_t SPDR = 5; //D1 blue
uint8_t ENCR = 12; //D6 yellow
uint8_t DIRRB = 0; //D3 white
uint8_t DIRRF = 2; //D4 white

float Vl_goal=0.0;
float Vr_goal=0.0;
float V_goal=0.0;
float W_goal=0.0;
float _V=0.0;
float _W=0.0;

const char* ssid = "NETGEAR";
const uint16_t serverPort = 11411;
std_msgs::String str_msg;
geometry_msgs::Twist vl_msg;

void check_node_connect();
IRAM_ATTR void cntL();
IRAM_ATTR void cntR();
void motor_running();

float pwmsLF[ti];
float pwmsLB[ti];
float pwmsRF[ti];
float pwmsRB[ti];

void twistcb( const geometry_msgs::Twist& twstmsg);
void messageCb( const std_msgs::String& toggle_msg){
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
float getOmega(float _vl, float _vr){
  return (((_vr-_vl)/L)*R);
}
float getLinear(float _vl, float _vr){
  return (((_vr+_vl)/2)*R);
}
float getVr(float Vx, float W){
  return (((2*Vx)+(W*L))/(2*R));
}
float getVl(float Vx, float W){
  return (((2*Vx)-(W*L))/(2*R));
}
float getMetersPerSecondLinear(){
  return (Pi_Rsqr_div_tiks_full_round*(tachoL.getHz()+tachoR.getHz()));
}
float getMetersPerSecondAngular(){
  return (omegaMultiplier*(tachoR.getHz()-tachoL.getHz()));
}
void leftFWD()
{
digitalWrite(DIRLF,0);
digitalWrite(DIRLB,0);
}
void leftBWD()
{
digitalWrite(DIRLF,1);
digitalWrite(DIRLB,1);
}
void rightFWD()
{
digitalWrite(DIRRF,0);
digitalWrite(DIRRB,0);
}
void rightBWD()
{
digitalWrite(DIRRF,1);
digitalWrite(DIRRB,1);
}
void rightSTP()
{
  digitalWrite(SPDR,HIGH);
  digitalWrite(DIRRF,LOW);
  digitalWrite(DIRRB,LOW);
}
void leftSTP()
{
  digitalWrite(SPDL,HIGH);
  digitalWrite(DIRLF,LOW);
  digitalWrite(DIRLB,LOW);
}
void twistcb( const geometry_msgs::Twist& twstmsg){
  if(twstmsg.linear.x>0.0){
    if(twstmsg.linear.x>maxLinearSpeedFWD){
      Vl_goal=getVl(maxLinearSpeedFWD,twstmsg.angular.z);
      Vr_goal=getVr(maxLinearSpeedFWD,twstmsg.angular.z);
    }
  }
  else if(twstmsg.linear.x<0.0){
    if(twstmsg.linear.x<maxLinearSpeedBWD){
      Vl_goal=getVl(maxLinearSpeedBWD,twstmsg.angular.z);
      Vr_goal=getVr(maxLinearSpeedBWD,twstmsg.angular.z);
    }
  }
  else{
        Vl_goal=getVl(twstmsg.linear.x,twstmsg.angular.z);
        Vr_goal=getVr(twstmsg.linear.x,twstmsg.angular.z);      
    }
}
 
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
  pinMode(ENCL,INPUT_PULLUP);
  pinMode(ENCR,INPUT_PULLUP);
  analogWriteFreq(40000);
  attachInterrupt(digitalPinToInterrupt(ENCL), cntL, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENCR), cntR, FALLING);
  pinMode(SPDL,OUTPUT);
  pinMode(SPDR,OUTPUT);
  pinMode(DIRRF,OUTPUT);
  pinMode(DIRRB,OUTPUT);
  pinMode(DIRLF,OUTPUT);
  pinMode(DIRLB,OUTPUT);
}

void loop()
{
  motor_running();
  //check_node_connect();
  publishSpeeds();
  delay(10);
}
uint8_t mps_to_pwmLF(float mps){
  return (valuesLF[0]*mps+valuesLF[1]);
}
uint8_t mps_to_pwmRF(float mps){
  return (valuesRF[0]*mps+valuesRF[1]);
}
uint8_t mps_to_pwmLB(float mps){
  return(valuesLB[0]*mps+valuesLB[1]);
}
uint8_t mps_to_pwmRB(float mps){
  return(valuesRB[0]*mps+valuesRB[1]);
}
void motor_running(){
  if((Vl_goal>0)&&(Vr_goal>0)){
    rightBWD();
    leftFWD();
    analogWrite(SPDR,mps_to_pwmRB(Vr_goal));
    analogWrite(SPDL,mps_to_pwmLF(Vl_goal));
  }
  else if((Vl_goal<0)&&(Vr_goal<0)){
    rightFWD();
    leftBWD();
    analogWrite(SPDR,mps_to_pwmRF(Vr_goal));
    analogWrite(SPDL,mps_to_pwmLB(Vl_goal));
  }
  else if((Vl_goal>0)&&(Vr_goal<0)){
    rightFWD();
    leftFWD();
    analogWrite(SPDR,mps_to_pwmRF(Vr_goal));
    analogWrite(SPDL,mps_to_pwmLF(Vl_goal));
  }
  else if((Vl_goal<0)&&(Vr_goal>0)){
    rightBWD();
    leftBWD();
    analogWrite(SPDR,mps_to_pwmRB(Vr_goal));
    analogWrite(SPDL,mps_to_pwmLB(Vl_goal));
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
void publishSpeeds(){
  float mpsLnr=getMetersPerSecondLinear();
  if (Vl_goal<0){
    mpsLnr*=(-1);
  }
  FloatToMsg("Linear:",mpsLnr,6);
}
IRAM_ATTR void cntR(){
    tachoR.tick();
}
IRAM_ATTR void cntL() {
    tachoL.tick();
}
 
void getMaxSpeed()
{
  /*
   * 
float maxLinearSpeedBWD=0.2; //meters per second
float maxAngularSpeed=0.2; //radians per second
float maxLeftWheelFWD=0.2; //meters per second
float maxLeftWheelBWD=0.2; //meters per second
float maxRightWheelFWD=0.2; //meters per second
float maxRightWheelBWD=0.2; //meters per second
   */
  
  Serial.println();
  Serial.println("Getting max speeds");
  rightBWD();
  leftFWD();
  analogWrite(SPDR,0);
  analogWrite(SPDL,0);
  delayMicroseconds(1000000);
  yield();
  maxLeftWheelFWD=(tachoL.getHz()/tiks_full_round)*wheel_scope;
  maxRightWheelBWD=(tachoR.getHz()/tiks_full_round)*wheel_scope;
  leftSTP();
  rightSTP();
  if(maxLeftWheelFWD > maxRightWheelBWD){maxLinearSpeedFWD=maxRightWheelBWD;}
  else{maxLinearSpeedFWD=maxLeftWheelFWD;}
  delayMicroseconds(1000000);
  yield();
  rightFWD();
  leftBWD();
  analogWrite(SPDR,0);
  analogWrite(SPDL,0);
  delayMicroseconds(1000000);
  yield();
  maxLeftWheelBWD=(tachoL.getHz()/tiks_full_round)*wheel_scope;
  maxRightWheelFWD=(tachoR.getHz()/tiks_full_round)*wheel_scope;
  leftSTP();
  rightSTP();
  if(maxLeftWheelBWD > maxRightWheelFWD){maxLinearSpeedBWD=maxRightWheelFWD*(-1);}
  else{maxLinearSpeedBWD=maxLeftWheelBWD*(-1);}
  delayMicroseconds(1000000);
  yield();
  Serial.print("maxLeftWheelFWD:");
  Serial.println(maxLeftWheelFWD);
  Serial.print("maxLeftWheelBWD:");
  Serial.println(maxLeftWheelBWD);
  Serial.print("maxRightWheelFWD:");
  Serial.println(maxRightWheelFWD);
  Serial.print("maxRightWheelBWD:");
  Serial.println(maxRightWheelBWD);
  
  Serial.print("maxLinearSpeedBWD:");
  Serial.println(maxLinearSpeedBWD);
  Serial.print("maxLinearSpeedFWD:");
  Serial.println(maxLinearSpeedFWD);
  maxAngularSpeedCW = ((maxRightWheelBWD - maxLeftWheelFWD)*R)/L;  
  maxAngularSpeedCCW = ((maxRightWheelFWD - maxLeftWheelBWD)*R)/L;
  
  Serial.print("maxAngularSpeedCW:");
  Serial.println(maxAngularSpeedCW);
  Serial.print("maxAngularSpeedCCW:");
  Serial.println(maxAngularSpeedCCW);
  
  unsigned long t0=0;
  int pw=0;
  
  for(int i=0;i<ti;i++){
   rightBWD();
   leftFWD();
   pw=i*15;
   analogWrite(SPDR,pw);
   analogWrite(SPDL,pw);
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
   printFloat(pwmsLF[i],6);
   Serial.println();
   Serial.print(pw);
   Serial.print(" RB:");
   printFloat(pwmsRB[i],6);
   leftSTP();
   rightSTP();
   delayMicroseconds(1000000);
   yield();
   Serial.println();
   
   rightFWD();
   leftBWD();
   analogWrite(SPDR,pw);
   analogWrite(SPDL,pw);
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
   printFloat(pwmsLB[i],6);
   Serial.println();
   Serial.print(pw);
   Serial.print(" RF:");
   printFloat(pwmsRF[i],6);
   leftSTP();
   rightSTP();
   delayMicroseconds(1000000);
   yield();
   Serial.println();
   }
   linearRegressionOfPWMS();
}
void linearRegressionOfPWMS()
{
  for(int i=0;i<ti;i++){
   int pw=i*15;
   lr.learn(pwmsRF[i],pw);
   Serial.print(pw);
   Serial.print(" RF:");
   Serial.println(pwmsRF[i]);
   printFloat(pwmsRF[i],6);
   Serial.println();
  }
    Serial.print("Result: 0.12 -> ");
    printFloat(lr.calculate(0.12),6);

    Serial.print("Result: 0.01 -> ");
    printFloat(lr.calculate(0.01),6);

    Serial.print("Correlation: ");
    Serial.println(lr.correlation());

    Serial.print("Values: ");
    lr.parameters(valuesRF);
    Serial.print("Y = ");
    Serial.print(valuesRF[0]);
    Serial.print("*X + ");
    Serial.println(valuesRF[1]);
    lr.reset();

  for(int i=0;i<ti;i++){
   int pw=i*15;
   lr.learn(pwmsRB[i],pw);
   Serial.print(pw);
   Serial.print(" RB:");
   Serial.println(pwmsRB[i]);
   printFloat(pwmsRB[i],6);
   Serial.println();
  }
    Serial.println("End learn");

    Serial.print("Result: 0.12 -> ");
    printFloat(lr.calculate(0.12),6);

    Serial.print("Result: 0.01 -> ");
    printFloat(lr.calculate(0.01),6);

    Serial.print("Correlation: ");
    Serial.println(lr.correlation());

    Serial.print("Values: ");
    lr.parameters(valuesRB);
    Serial.print("Y = ");
    Serial.print(valuesRB[0]);
    Serial.print("*X + ");
    Serial.println(valuesRB[1]);
    lr.reset();

 //------------------------------------------//
   
  for(int i=0;i<ti;i++){
   int pw=i*15;
   lr.learn(pwmsLF[i],pw);
   Serial.print(pw);
   Serial.print(" LF:");
   Serial.println(pwmsLF[i]);
   printFloat(pwmsLF[i],6);
   Serial.println();
  }
    Serial.println("End learn");

    Serial.print("Result: 0.12 -> ");
    printFloat(lr.calculate(0.12),6);

    Serial.print("Result: 0.01 -> ");
    printFloat(lr.calculate(0.01),6);

    Serial.print("Correlation: ");
    Serial.println(lr.correlation());

    Serial.print("Values: ");
    lr.parameters(valuesLF);
    Serial.print("Y = ");
    Serial.print(valuesLF[0]);
    Serial.print("*X + ");
    Serial.println(valuesLF[1]);
    lr.reset();

  for(int i=0;i<ti;i++){
   int pw=i*15;
   lr.learn(pwmsLB[i],pw);
   Serial.print(pw);
   Serial.print(" LB:");
   Serial.println(pwmsLB[i]);
   printFloat(pwmsLB[i],6);
   Serial.println();
  }
    Serial.println("End learn");

    Serial.print("Result: 0.12 -> ");
    printFloat(lr.calculate(0.12),6);

    Serial.print("Result: 0.01 -> ");
    printFloat(lr.calculate(0.01),6);

    Serial.print("Correlation: ");
    Serial.println(lr.correlation());

    Serial.print("Values: ");
    lr.parameters(valuesLB);
    Serial.print("Y = ");
    Serial.print(valuesLB[0]);
    Serial.print("*X + ");
    Serial.println(valuesLB[1]);
    lr.reset();
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
void printFloat(double number, uint8_t digits) 
{ 
  // Handle negative numbers
  if (number < 0.0)
  {
     Serial.print('-');
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
  Serial.print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    Serial.print("."); 

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    Serial.print(toPrint);
    remainder -= toPrint; 
  } 
}
