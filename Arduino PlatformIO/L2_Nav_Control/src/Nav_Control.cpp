/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nh;
geometry_msgs::Twist msg;

float steer;

int motor1pin1 = 2;
int motor1pin2 = 3;

int motor2pin1 = 4;
int motor2pin2 = 5;

void messageCb( const geometry_msgs::Twist& msg){
  steer = msg.angular.z;
  analogWrite(9, steer*1000); //ENA   pin
  analogWrite(10, steer*1000); //ENB pin
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", messageCb );
void setup()
{
  // put your setup code here, to run once:
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1,   OUTPUT);
  pinMode(motor2pin2, OUTPUT);

  //(Optional)
  pinMode(9,   OUTPUT); 
  pinMode(10, OUTPUT);
  //(Optional)
  //Controlling speed (0   = off and 255 = max speed):     
  //(Optional)
  
  //(Optional)
  
  nh.initNode();
  nh.subscribe(sub);
  Serial.begin(57600);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}