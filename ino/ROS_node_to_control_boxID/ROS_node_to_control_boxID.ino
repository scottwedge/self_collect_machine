#include <ros.h>
#include <std_msgs/Int32.h>

#define boxID1 13
//#define boxID2 50
//#define boxID3 52

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
}

ros::Subscriber<std_msgs::Int32> sub("/boxID_activation", messageCb);

// put your setup code here, to run once:
void setup()
{
  //configure pins as an input and enable the internal pull-up resistor
  pinMode(boxID1, OUTPUT);
//  pinMode(boxID2, OUTPUT);
//  pinMode(boxID3, OUTPUT);
  
  nh.initNode();
  
  nh.subscribe(sub);
}

// put your main code here, to run repeatedly:
void loop()
{  
  nh.spinOnce();
  delay(10);
}
