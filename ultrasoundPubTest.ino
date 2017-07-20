#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include "Arduino.h"

ros::NodeHandle nh;

sensor_msgs::Range range_msg;
ros::Publisher pub_range1("ultrasound", &range_msg);

char frameid[] = "/ultrasound";

long range_time;
long duration;
float getRange_Ultrasound(int pin_num)
{
    pinMode(pin_num, OUTPUT);
    digitalWrite(pin_num, LOW);
    delayMicroseconds(2);
    digitalWrite(pin_num, HIGH);
    delayMicroseconds(10);
    digitalWrite(pin_num, LOW);
    
    pinMode(pin_num, INPUT);
    duration = pulseIn(pin_num, HIGH);
    
    return duration/58;
}

void setup()
{
    nh.initNode();
    nh.advertise(pub_range1);
    
    range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msg.header.frame_id = frameid;
    range_msg.field_of_view = 0.1;
    range_msg.min_range = 0.0;
    range_msg.max_range = 2.0;
}

void loop()
{
    if ( (millis()-range_time) > 50)
    {
        range_msg.range = getRange_Ultrasound(7);
        range_msg.header.stamp = nh.now();
        pub_range1.publish(&range_msg);
        range_time = millis();
    }
    nh.spinOnce();
}
