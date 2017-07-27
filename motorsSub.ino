#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>

#define A_DIR 12
#define A_BRK 8
#define A_PWM 3
#define B_DIR 13
#define B_BRK 9
#define B_PWM 11

ros::NodeHandle nh;
int commandL = 0;
int commandR = 0;

void motorsCbL(const std_msgs::Float64& msg)
{
    commandL = msg.data;
    
    if (commandL == 0.0)
    {
        digitalWrite(A_DIR, LOW);
        analogWrite(A_PWM, commandL);
    }
    else if (commandL < 0)
    {
        //Forward: Direction = LOW; Backward: Direction = HIGH
        digitalWrite(A_DIR, HIGH);
        analogWrite(A_PWM, commandL);
    }
    else
    {
        digitalWrite(A_DIR, LOW);
        analogWrite(A_PWM, commandL);
    }
}

void motorsCbR(const std_msgs::Float64& msg)
{
    commandR = msg.data;
    
    if (commandR == 0.0)
    {
        digitalWrite(B_DIR, LOW);
        analogWrite(B_PWM, commandR);
    }
    else if (commandR < 0)
    {
        //Forward: Direction = LOW; Backward: Direction = HIGH
        digitalWrite(B_DIR, HIGH);
        analogWrite(B_PWM, commandR);
    }
    else
    {
        digitalWrite(B_DIR, LOW);
        analogWrite(B_PWM, commandR);
    }
}

ros::Subscriber<std_msgs::Float64> sub_left("leftMotors", &motorsCbL);
ros::Subscriber<std_msgs::Float64> sub_right("rightMotors", &motorsCbR);


void setup()
{
    pinMode(A_DIR, OUTPUT);
    pinMode(A_BRK, OUTPUT);
    pinMode(A_PWM, OUTPUT);
    pinMode(B_DIR, OUTPUT);
    pinMode(B_BRK, OUTPUT);
    pinMode(B_PWM, OUTPUT);
    
    nh.initNode();
    nh.subscribe(sub_left);
    nh.subscribe(sub_right);
    
    digitalWrite(A_DIR, LOW);
    analogWrite(A_PWM, 0);
    digitalWrite(B_DIR, LOW);
    analogWrite(B_PWM, 0);
}

void loop()
{
    nh.spinOnce();
    delay(20);
}
