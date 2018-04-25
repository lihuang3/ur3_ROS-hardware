/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Empty.h>
#include <ur5_notebook/Tracker.h>

ros::NodeHandle  nh;

void messageCb( const ur5_notebook::Tracker & msg){
  if (msg.flag2) 
  {
    digitalWrite(13, LOW);
    Serial.println("Gripper on!");
  }
  else 
  {
    digitalWrite(13, HIGH); // toggle ssr 
    Serial.println("Gripper off!");
  }
}

ros::Subscriber<ur5_notebook::Tracker> sub("cxy1", &messageCb);

void setup()
{ 
  Serial.begin(57600);  
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}

