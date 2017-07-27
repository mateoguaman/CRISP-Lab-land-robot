#include <ros.h>
#include <ros/time.h>
#include <Wire.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>

#define A_DIR 12
#define A_BRK 8
#define A_PWM 3
#define B_DIR 13
#define B_BRK 9
#define B_PWM 11

ros::NodeHandle nh;

Servo servo;

int commandL = 0;
int commandR = 0;

sensor_msgs::Range range_msg;
sensor_msgs::MagneticField mag_msg;
sensor_msgs::Imu imu_msg;
geometry_msgs::Vector3Stamped rpy_msg;
geometry_msgs::Vector3 accel_vec;
geometry_msgs::Vector3 gyro_vec;
geometry_msgs::Vector3 mag_vec;
geometry_msgs::Vector3 rpy_vec;

char frameidUS[] = "/ultrasound";
char frameidMag[] = "/magnetometer";
char frameidImu[] = "/imu";
char frameidRpy[] = "/rpy";

Adafruit_9DOF dof = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);

long range_time;
long duration;

float roll;
float pitch;
float heading;
float accelX;
float accelY;
float accelZ;
float gyroX;
float gyroY;
float gyroZ;
float magX;
float magY;
float magZ;

void initSensors()
{

    accel.enableAutoRange(true);
    if(!accel.begin())
    {
        /* There was a problem detecting the LSM303 ... check your connections */
        Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
        while(1);
    }

    mag.enableAutoRange(true);
    if(!mag.begin())
    {
        /* There was a problem detecting the LSM303 ... check your connections */
        Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
        while(1);
    }

    gyro.enableAutoRange(true);
    if(!gyro.begin())
    {
        /* There was a problem detecting the L3GD20 ... check your connections */
        Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
        while(1);
    }
}

void servo_cb(const std_msgs::UInt16& cmd_msg)
{
    servo.write(cmd_msg.data);
}

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

ros::Subscriber<std_msgs::UInt16> sub_servo("servo", servo_cb);
ros::Subscriber<std_msgs::Float64> sub_left("leftMotors", &motorsCbL);
ros::Subscriber<std_msgs::Float64> sub_right("rightMotors", &motorsCbR);
ros::Publisher pub_mag("magnetometer", &mag_msg);
ros::Publisher pub_imu("imu", &imu_msg);
ros::Publisher pub_rpy("rpy", &rpy_msg);
ros::Publisher pub_range("ultrasound", &range_msg);

void setup()
{
    //Serial.begin(57600);
    pinMode(A_DIR, OUTPUT);
    pinMode(A_BRK, OUTPUT);
    pinMode(A_PWM, OUTPUT);
    pinMode(B_DIR, OUTPUT);
    pinMode(B_BRK, OUTPUT);
    pinMode(B_PWM, OUTPUT);
    
    nh.initNode();
    nh.advertise(pub_range);
    nh.advertise(pub_mag);
    nh.advertise(pub_imu);
    nh.advertise(pub_rpy);
    nh.subscribe(sub_servo);
    nh.subscribe(sub_left);
    nh.subscribe(sub_right);
    
    initSensors();
    servo.attach(10);
    
    digitalWrite(A_DIR, LOW);
    analogWrite(A_PWM, 0);
    digitalWrite(B_DIR, LOW);
    analogWrite(B_PWM, 0);
    
    range_msg.header.frame_id = frameidUS;
    mag_msg.header.frame_id = frameidMag;
    imu_msg.header.frame_id = frameidImu;
    rpy_msg.header.frame_id = frameidRpy;
    
    range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msg.field_of_view = 0.1;
    range_msg.min_range = 0.02;
    range_msg.max_range = 3.0;
}

void loop()
{
    sensors_event_t accel_event;
    sensors_event_t mag_event;
    sensors_event_t gyro_event;
    sensors_vec_t orientation;
    
    accel.getEvent(&accel_event);
    accelX = accel_event.acceleration.x;
    accelY = accel_event.acceleration.y;
    accelZ = accel_event.acceleration.z;
    
    gyro.getEvent(&gyro_event);
    gyroX = gyro_event.gyro.x;
    gyroY = gyro_event.gyro.y;
    gyroZ = gyro_event.gyro.z;
    
    mag.getEvent(&mag_event);
    magX = mag_event.magnetic.x;
    magY = mag_event.magnetic.y;
    magZ = mag_event.magnetic.z;
    
    if (dof.accelGetOrientation(&accel_event, &orientation))
    {
        roll = orientation.roll;
        pitch = orientation.pitch;
    }
    
    if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
    {
        heading = orientation.heading;
    }
    
    accel_vec.x = accelX;
    accel_vec.y = accelY;
    accel_vec.z = accelZ;
    
    gyro_vec.x = gyroX;
    gyro_vec.y = gyroY;
    gyro_vec.z = gyroZ;
    
    mag_vec.x = magX;
    mag_vec.y = magY;
    mag_vec.z = magZ;
    
    rpy_vec.x = roll;
    rpy_vec.y = pitch;
    rpy_vec.z = heading;
    
    imu_msg.angular_velocity = gyro_vec;
    imu_msg.linear_acceleration = accel_vec;
    imu_msg.header.stamp = nh.now();
    mag_msg.header.stamp = nh.now();
    rpy_msg.header.stamp = nh.now();
    mag_msg.magnetic_field = mag_vec;
    rpy_msg.vector = rpy_vec;
    
    if ((millis()-range_time) > 50)
    {
        range_msg.range = getRange_Ultrasound(7);
        range_msg.header.stamp = nh.now();
        pub_range.publish(&range_msg);
        range_time = millis();
    }
    pub_mag.publish(&mag_msg);
    pub_imu.publish(&imu_msg);
    pub_rpy.publish(&rpy_msg);
    
    nh.spinOnce();    
}


