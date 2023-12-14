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
