/*
################################################
## {Description}: Code (Bar/QR) Recognition 
## using USB type camera
################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################
 */
 
#include <ros.h>
#include <std_msgs/Int32.h>

// add more sensor here
#define sensor1 2
#define sensor2 3
//#define sensorN xx

// add more boxID here
#define boxID1 9
//#define boxID2 3
//#define boxID3 52

// add more boxIDswitch here
#define boxIDswitch1 8
//#define boxIDswitch2 3
//#define boxIDswitch3 52

int sensorState;

ros::NodeHandle nh;

//this function control boxID activation
void messageCb(const std_msgs::Int32 &msg)
{
  int boxID = msg.data;
  
  if(boxID == 0)
  {
    if (digitalRead(boxIDswitch1) == LOW)
    {
      digitalWrite(boxID1, HIGH);
      delay(1000);
      digitalWrite(boxID1, LOW);
      delay(1000); 
    }
  }

//  else if(boxID == 1)
//  {
//    digitalWrite(boxID2, HIGH);
//    delay(1000);
//    digitalWrite(boxID2, LOW);
//    delay(1000);
//    digitalWrite(boxID2, HIGH);
//    delay(1000);
//    digitalWrite(boxID2, LOW);
//    delay(1000);
//    digitalWrite(boxID2, HIGH);
//    delay(1000);
//    digitalWrite(boxID2, LOW);
//    delay(1000);
//    digitalWrite(boxID2, HIGH);
//    delay(1000);
//    digitalWrite(boxID2, LOW);
//    delay(1000);
//  }
}

//this function returns the sensor state
int sensor_state(int sense)
{
  sensorState = digitalRead(sense);
  return sensorState;
}

std_msgs::Int32 sensorState_1_msg;
std_msgs::Int32 sensorState_2_msg;
//std_msgs::Int32 sensorState_3_msg;

ros::Publisher pub_sensor1("/sensorState_1", &sensorState_1_msg);
ros::Publisher pub_sensor2("/sensorState_2", &sensorState_2_msg);
//ros::Publisher pub_sensor3("/sensorState_3", &sensorState_3_msg);

std_msgs::Int32 switchState_1_msg;

ros::Publisher pub_boxIDswitch1("/switchState_1", &switchState_1_msg);

ros::Subscriber<std_msgs::Int32> sub("/boxID_activation", messageCb);

// put your setup code here, to run once:
void setup()
{
  //configure pins as an input and enable the internal pull-up resistor
  pinMode(sensor1, INPUT_PULLUP);
  pinMode(sensor2, INPUT_PULLUP);
//  pinMode(sensor3, INPUT_PULLUP);

  pinMode(boxID1, OUTPUT);
//  pinMode(boxID2, OUTPUT);
//  pinMode(boxID3, OUTPUT); 

  pinMode(boxIDswitch1, INPUT_PULLUP);
  
  nh.initNode();
  
  nh.advertise(pub_sensor1);
  nh.advertise(pub_sensor2);
//  nh.advertise(pub_sensor3);

  nh.advertise(pub_boxIDswitch1);

  nh.subscribe(sub);
}

// put your main code here, to run repeatedly:
void loop()
{
  sensorState_1_msg.data=sensor_state(sensor1);
  pub_sensor1.publish(&sensorState_1_msg);

  sensorState_2_msg.data=sensor_state(sensor2);
  pub_sensor2.publish(&sensorState_2_msg);

//  sensorState_3_msg.data=sensor_state(sensor3);
//  pub_sensor3.publish(&sensorState_3_msg);

  switchState_1_msg.data=sensor_state(boxIDswitch1);
  pub_boxIDswitch1.publish(&switchState_1_msg);
  
  nh.spinOnce();
  delay(10);
}
