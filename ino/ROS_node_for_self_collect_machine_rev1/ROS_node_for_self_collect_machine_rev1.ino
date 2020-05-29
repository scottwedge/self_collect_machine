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

// sensor pin numbering
#define sensor1 8
#define sensor2 9
#define sensor3 10
#define sensor4 11
#define sensor5 12
#define sensor6 13

// lock@relay pin numbering
#define boxID1 2
#define boxID2 3
#define boxID3 4
#define boxID4 5
#define boxID5 6
#define boxID6 7

// lock@relay switch pin numbering
//#define boxID1switch A0
//#define boxID2switch A1
//#define boxID3switch A2
//#define boxID4switch A3
//#define boxID5switch A4
//#define boxID6switch A5

int sensorState;

ros::NodeHandle nh;

//this function control boxID activation
void messageCb(const std_msgs::Int32 &msg)
{
  int boxID = msg.data;
  
  if(boxID == 0)
  {
    digitalWrite(boxID1, HIGH);
    delay(1000);
    digitalWrite(boxID1, LOW);
    delay(1000);
    digitalWrite(boxID1, HIGH);
    delay(1000);
    digitalWrite(boxID1, LOW);
    delay(1000); 
  }

  else if(boxID == 1)
  {
    digitalWrite(boxID2, HIGH);
    delay(1000);
    digitalWrite(boxID2, LOW);
    delay(1000);
    digitalWrite(boxID2, HIGH);
    delay(1000);
    digitalWrite(boxID2, LOW);
    delay(1000); 
  }

    else if(boxID == 2)
  {
    digitalWrite(boxID3, HIGH);
    delay(1000);
    digitalWrite(boxID3, LOW);
    delay(1000);
    digitalWrite(boxID3, HIGH);
    delay(1000);
    digitalWrite(boxID3, LOW);
    delay(1000); 
  }

}

//this function returns the sensor state
int sensor_state(int sense)
{
  sensorState = digitalRead(sense);
  return sensorState;
}

std_msgs::Int32 sensorState_1_msg;
std_msgs::Int32 sensorState_2_msg;
std_msgs::Int32 sensorState_3_msg;
std_msgs::Int32 sensorState_4_msg;
std_msgs::Int32 sensorState_5_msg;
std_msgs::Int32 sensorState_6_msg;

ros::Publisher pub_sensor1("/sensorState_1", &sensorState_1_msg);
ros::Publisher pub_sensor2("/sensorState_2", &sensorState_2_msg);
ros::Publisher pub_sensor3("/sensorState_3", &sensorState_3_msg);
ros::Publisher pub_sensor4("/sensorState_4", &sensorState_4_msg);
ros::Publisher pub_sensor5("/sensorState_5", &sensorState_5_msg);
ros::Publisher pub_sensor6("/sensorState_6", &sensorState_6_msg);

//std_msgs::Int32 switchState_1_msg;
//std_msgs::Int32 switchState_2_msg;
//std_msgs::Int32 switchState_3_msg;
//std_msgs::Int32 switchState_4_msg;
//std_msgs::Int32 switchState_5_msg;
//std_msgs::Int32 switchState_6_msg;
//
//ros::Publisher pub_boxIDswitch1("/switchState_1", &switchState_1_msg);
//ros::Publisher pub_boxIDswitch2("/switchState_2", &switchState_2_msg);
//ros::Publisher pub_boxIDswitch3("/switchState_3", &switchState_3_msg);
//ros::Publisher pub_boxIDswitch4("/switchState_4", &switchState_4_msg);
//ros::Publisher pub_boxIDswitch5("/switchState_5", &switchState_5_msg);
//ros::Publisher pub_boxIDswitch6("/switchState_6", &switchState_6_msg);

ros::Subscriber<std_msgs::Int32> sub("/boxID_activation", messageCb);

// put your setup code here, to run once:
void setup()
{
  //configure pins as an input and enable the internal pull-up resistor
  pinMode(sensor1, INPUT_PULLUP);
  pinMode(sensor2, INPUT_PULLUP);
  pinMode(sensor3, INPUT_PULLUP);
  pinMode(sensor4, INPUT_PULLUP);
  pinMode(sensor5, INPUT_PULLUP);
  pinMode(sensor6, INPUT_PULLUP);

  pinMode(boxID1, OUTPUT);
  pinMode(boxID2, OUTPUT);
  pinMode(boxID3, OUTPUT);
  pinMode(boxID4, OUTPUT);
  pinMode(boxID5, OUTPUT);
  pinMode(boxID6, OUTPUT); 

//  pinMode(boxID1switch, INPUT_PULLUP);
//  pinMode(boxID2switch, INPUT_PULLUP);
//  pinMode(boxID3switch, INPUT_PULLUP);
//  pinMode(boxID4switch, INPUT_PULLUP);
//  pinMode(boxID5switch, INPUT_PULLUP);
//  pinMode(boxID6switch, INPUT_PULLUP);
  
  nh.initNode();
  
  nh.advertise(pub_sensor1);
  nh.advertise(pub_sensor2);
  nh.advertise(pub_sensor3);
  nh.advertise(pub_sensor4);
  nh.advertise(pub_sensor5);
  nh.advertise(pub_sensor6);

//  nh.advertise(pub_boxIDswitch1);
//  nh.advertise(pub_boxIDswitch2);
//  nh.advertise(pub_boxIDswitch3);
//  nh.advertise(pub_boxIDswitch4);
//  nh.advertise(pub_boxIDswitch5);
//  nh.advertise(pub_boxIDswitch6);

  nh.subscribe(sub);
}

// put your main code here, to run repeatedly:
void loop()
{
  sensorState_1_msg.data=sensor_state(sensor1);
  pub_sensor1.publish(&sensorState_1_msg);

  sensorState_2_msg.data=sensor_state(sensor2);
  pub_sensor2.publish(&sensorState_2_msg);

  sensorState_3_msg.data=sensor_state(sensor3);
  pub_sensor3.publish(&sensorState_3_msg);

  sensorState_4_msg.data=sensor_state(sensor4);
  pub_sensor4.publish(&sensorState_4_msg);

  sensorState_5_msg.data=sensor_state(sensor5);
  pub_sensor5.publish(&sensorState_5_msg);

  sensorState_6_msg.data=sensor_state(sensor6);
  pub_sensor6.publish(&sensorState_6_msg);

//  switchState_1_msg.data=sensor_state(boxID1switch);
//  pub_boxIDswitch1.publish(&switchState_1_msg);
//
//  switchState_2_msg.data=sensor_state(boxID2switch);
//  pub_boxIDswitch2.publish(&switchState_2_msg);
//
//  switchState_3_msg.data=sensor_state(boxID3switch);
//  pub_boxIDswitch3.publish(&switchState_3_msg);
//
//  switchState_4_msg.data=sensor_state(boxID4switch);
//  pub_boxIDswitch4.publish(&switchState_4_msg);
//
//  switchState_5_msg.data=sensor_state(boxID5switch);
//  pub_boxIDswitch5.publish(&switchState_5_msg);
//
//  switchState_6_msg.data=sensor_state(boxID6switch);
//  pub_boxIDswitch6.publish(&switchState_6_msg);
  
  nh.spinOnce();
  delay(10);
}
