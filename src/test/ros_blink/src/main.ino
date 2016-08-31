/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

//https://defendtheplanet.net/2014/05/22/arduino-microleonardo-and-rosserial-hello-world-possible-link-problem/
//Fix for Leonardo?
#define USBCON
//#define USE_USBCON

#include <ros.h>
#include <std_msgs/Empty.h>

ros::NodeHandle nh;

void messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );

void setup()
{
  pinMode(13, OUTPUT);
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}