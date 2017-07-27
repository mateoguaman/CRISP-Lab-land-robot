#include <ros.h>
#include <ros/time.h>
#include <Wire.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Imu.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>

ros::NodeHandle nh;

sensor_msgs::MagneticField mag_msg;
sensor_msgs::Imu imu_msg;
geometry_msgs::Vector3Stamped rpy_msg;
geometry_msgs::Vector3 accel_vec;
geometry_msgs::Vector3 gyro_vec;
geometry_msgs::Vector3 mag_vec;
geometry_msgs::Vector3 rpy_vec;


char frameidMag[] = "/magnetometer";
char frameidImu[] = "/imu";
char frameidRpy[] = "/rpy";

Adafruit_9DOF dof = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);

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

ros::Publisher pub_mag("magnetometer", &mag_msg);
ros::Publisher pub_imu("imu", &imu_msg);
ros::Publisher pub_rpy("rpy", &rpy_msg);


void setup()
{
    Serial.begin(57600);
    nh.initNode();
    nh.advertise(pub_mag);
    nh.advertise(pub_imu);
    nh.advertise(pub_rpy);
    
    initSensors();
    
    mag_msg.header.frame_id = frameidMag;
    imu_msg.header.frame_id = frameidImu;
    rpy_msg.header.frame_id = frameidRpy;
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
    
    
    pub_mag.publish(&mag_msg);
    pub_imu.publish(&imu_msg);
    pub_rpy.publish(&rpy_msg);
    
    nh.spinOnce();    
}


