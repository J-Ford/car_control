#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>


#define MAX_EFFORT 10
#define DRIVE_FORWARD_COMMAND 4
#define DRIVE_BACKWARDS_COMMAND 5
#define STEER_RIGHT_COMMAND 0 
#define STEER_LEFT_COMMAND 1
#define MC_ADDRESS 128

#define lightPin 6


ros::NodeHandle rosNode;

void throttleMessageCb( const std_msgs::Float64& msg){
  if(msg.data < 0){
    sendMotorCommand(min(abs(msg.data),MAX_EFFORT),DRIVE_BACKWARDS_COMMAND);
  }
  else{
    sendMotorCommand(min(msg.data,MAX_EFFORT),DRIVE_FORWARD_COMMAND);
  }
}

void steeringMessageCb( const std_msgs::Float64& msg){
  if(msg.data < 0){
    sendMotorCommand(abs(msg.data),STEER_LEFT_COMMAND);    
  }
  else{
    sendMotorCommand(msg.data,STEER_RIGHT_COMMAND);
  }
}

void lightMessageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(lightPin, HIGH-digitalRead(lightPin));   // blink the led
}

ros::Subscriber<std_msgs::Float64> throttleSubscriber("throttle/control_effort/data", throttleMessageCb);
ros::Subscriber<std_msgs::Float64> steeringSubscriber("steering/control_effort/data", steeringMessageCb);
ros::Subscriber<std_msgs::Empty> lightSubscriber("toggle_light", lightMessageCb );


void setup(){
  pinMode(lightPin, OUTPUT);
  Serial1.begin(9600);
  rosNode.initNode();
  rosNode.subscribe(throttleSubscriber);
  rosNode.subscribe(steeringSubscriber);
  rosNode.subscribe(lightSubscriber);
}

void loop(){
  rosNode.spinOnce();
  delay(10);
}

void sendMotorCommand(int effort, int command){
  Serial1.write(MC_ADDRESS); //address (must be same as address specified on motor controller)
  Serial1.write(command);
  Serial1.write(effort);
  Serial1.write((MC_ADDRESS + command + effort) & 0b01111111); //checksum
}







