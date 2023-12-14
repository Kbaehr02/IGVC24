<<<<<<< HEAD
#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <Servo.h>

//=================
// L298 connections
//=================
// Left motor
#define input_1 2
#define input_2 3
#define ena  9
// Right motor
#define input_3  4
#define input_4  5
#define enb  10

//========================
// Ros object declarations
//========================
ros::NodeHandle  nh;
geometry_msgs::Twist msg_1;
std_msgs::Float32MultiArray msg_2, msg_3;

//===================================================
// Initialize dimension variables of differential bot
//===================================================
float track_width; // Distance between wheels in meters
float wheel_radius; // Radius of wheels in meters
float max_rpm; // Calculated max RPM of motors
float rpm_to_analog; // Maps rpm values from 0-255 
float steer; // Angular velocity in rad/s
float speed; // Speed in m/s
float rpm_left; // RPM of left motor
float rpm_right; // RPM of right motor

//=========================================================
// Callback function in the event a ROS message is received
//=========================================================
void cmd_velMsg( const geometry_msgs::Twist& msg_1){
  // Store ROS twist message and calculate RPM
  steer = msg_1.angular.z;
  speed = msg_1.linear.x;
  rpm_left = (60*(speed-(track_width*steer)))/(2*3.14*wheel_radius); 
  rpm_right = (60*(speed+(track_width*steer)))/(2*3.14*wheel_radius);
  // Apply control to motors
  analogWrite(ena, rpm_left*rpm_to_analog);
  analogWrite(enb, rpm_right*rpm_to_analog);
  digitalWrite(input_1, LOW);
  digitalWrite(input_2, HIGH);
  digitalWrite(input_3, HIGH);
  digitalWrite(input_4, LOW);
}

//====================================
// Receives message from python script
//====================================
void pythonCommsMsg(const std_msgs::Float32MultiArray& msg_2){
  track_width = msg_2.data[0];
  wheel_radius = msg_2.data[1];
  max_rpm = msg_2.data[2];
  rpm_to_analog = msg_2.data[3];
}

//=======================================
// Instantiate ROS subscribers/publishers
//=======================================
ros::Subscriber<geometry_msgs::Twist> sub_1("/cmd_vel", cmd_velMsg );
ros::Subscriber<std_msgs::Float32MultiArray> sub_2("comms/python", pythonCommsMsg );
ros::Publisher pub_1("/comms/arduino", &msg_3);

//==========================================
// Initial setup before looped main function
//==========================================
void setup()
{
  Serial.begin(57600);
  pinMode(input_1, OUTPUT);
  pinMode(input_2, OUTPUT);
  pinMode(input_3,   OUTPUT);
  pinMode(input_4, OUTPUT);
  pinMode(ena, OUTPUT); 
  pinMode(enb, OUTPUT);
  nh.initNode(); // Establish node to communicate via ROS
  nh.subscribe(sub_1);
  nh.subscribe(sub_2);
  nh.advertise(pub_1);
}

//=====================
// Looped main function
//=====================
void loop()
{
  float data_array[2] = {rpm_left, rpm_right};
  msg_3.data_length = 2;
  msg_3.data = data_array;
  pub_1.publish(&msg_3);
  nh.spinOnce();
}
=======
/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */
#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

ros::NodeHandle  nh;
geometry_msgs::Twist msg1;
std_msgs::Float64 msg2;

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
  /*analogWrite(ENA, rpmLeft*rpm2analog);
  analogWrite(ENB, rpmRight*rpm2analog);
  //Set direction of motors forward
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);*/
}
void pythonCommsMsg( const std_msgs::Float64& msg2){
  rpm2analog = msg2.data;
  Serial.println(rpm2analog);
}
ros::Subscriber<geometry_msgs::Twist> sub1("/cmd_vel", cmd_velMsg );
ros::Subscriber<std_msgs::Float64> sub2("comms/arduino", pythonCommsMsg );
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
  Serial.begin(9600);
}

void loop()
{
  //Serial.println(rpm2analog);
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
  //Set direction of motors forward
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
  nh.spinOnce();
}
>>>>>>> 1030b7c9969860025b533d0a6e6d8503ac73da37
