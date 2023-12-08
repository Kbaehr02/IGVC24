/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */
#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
ros::NodeHandle  nh;
geometry_msgs::Twist msg1;
std_msgs::Float64 msg2;
std_msgs::Float32 msg3;
//=====================================================
// Dimensions of differential bot and conversion equations
const float trackWidth = .35; // Distance between wheels in meters
const float wheelRadius = .0719/2; // Radius of wheel in meters
const float wheelCircumference = 2*PI*wheelRadius;
const float maxRPM = (((2.245/wheelCircumference)/16)*60); 
// 2.245 = test track dist & 16 = time to complete, 2.245/circumference = revolutions made
// revs/16 = RPS, RPS*60 = RPM. Calculations done at max speed. maxRPM will be used to map 
// desired RPM to a range of 255 for the analogWrite function.
float rpm2analog = 0;
//=====================================================
// Vaiable and pin definitions

float steer; // Angular velocity in rad/s
float speed; // Speed in m/s
int rpmLeft;
int rpmRight;

int motor1pin1 = 2;
int motor1pin2 = 3;
int ENA = 9;

int ENB = 10;
int motor2pin1 = 4;
int motor2pin2 = 5;

//=====================================================
// Callback function in the event a ROS message is received
void cmd_velMsg( const geometry_msgs::Twist& msg1){
  steer = msg1.angular.z; // Angular velocity of bot sent from ROS
  speed = msg1.linear.x;  // Speed of bot sent from ROS (magnitude)
  rpmLeft = int((60*(speed-(trackWidth*steer)))/(2*3.14*wheelRadius)); 
  rpmRight = int((60*(speed+(trackWidth*steer)))/(2*3.14*wheelRadius));
  //rpmLeft = (2*speed-steer*trackWidth)/(2*wheelRadius);
  //rpmRight = (2*speed+steer*trackWidth)/(2*wheelRadius);
  if (rpmLeft >= maxRPM){
    rpmLeft= maxRPM;
    rpmRight = maxRPM/5;
  }
  if (rpmRight >= maxRPM){
    rpmRight= maxRPM;
    rpmLeft = maxRPM/5;
  }
  analogWrite(ENA, rpmLeft*rpm2analog);
  analogWrite(ENB, rpmRight*rpm2analog);
  //Set direction of motors forward
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
}
void pythonCommsMsg( const std_msgs::Float64& msg2){
  rpm2analog = msg2.data;
}
ros::Subscriber<geometry_msgs::Twist> sub1("/cmd_vel", cmd_velMsg );
ros::Subscriber<std_msgs::Float64> sub2("comms/python", pythonCommsMsg );
//ros::Publisher sendData("comms/arduino", &msg3);
ros::Publisher sendData("/comms/arduino", &msg3);
//=====================================================
// Setup and looped function
void setup()
{
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1,   OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  pinMode(9, OUTPUT); 
  pinMode(10, OUTPUT);
  
  nh.initNode(); // Establish node to communicate via ROS
  nh.subscribe(sub1); // Register subscriber "sub" defined after message callback function
  nh.subscribe(sub2);
  nh.advertise(sendData);
  Serial.begin(57600);
}

void loop()
{
  msg3.data = rpm2analog*rpmRight;
  sendData.publish(&msg3);
  /*analogWrite(ENA, 255);
  analogWrite(ENB, 255);
  //Set direction of motors forward
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);*/
  nh.spinOnce();
}