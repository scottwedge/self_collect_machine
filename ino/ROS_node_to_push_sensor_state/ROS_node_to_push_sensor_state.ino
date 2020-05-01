#include <ros.h>
#include <std_msgs/Int32.h>

#define sensor1 48
#define sensor2 50
#define sensor3 52

int sensorState;

ros::NodeHandle nh;
 
//this function returns the sensor state
int sensor_state(int sense)
{
  sensorState = digitalRead(sense);
  return sensorState;
}

std_msgs::Int32 sensorState_1_msg;
std_msgs::Int32 sensorState_2_msg;
std_msgs::Int32 sensorState_3_msg;

ros::Publisher pub_sensor1("/sensorState_1", &sensorState_1_msg);
ros::Publisher pub_sensor2("/sensorState_2", &sensorState_2_msg);
ros::Publisher pub_sensor3("/sensorState_3", &sensorState_3_msg);

// put your setup code here, to run once:
void setup()
{
  //configure pins as an input and enable the internal pull-up resistor
  pinMode(sensor1, INPUT_PULLUP);
  pinMode(sensor2, INPUT_PULLUP);
  pinMode(sensor3, INPUT_PULLUP);
  
  nh.initNode();
  
  nh.advertise(pub_sensor1);
  nh.advertise(pub_sensor2);
  nh.advertise(pub_sensor3);
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
  
  nh.spinOnce();
  delay(10);
}
