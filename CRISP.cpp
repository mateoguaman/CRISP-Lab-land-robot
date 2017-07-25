#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>
#include <Servo.h>

//=================================================================
//                              SERVO
//=================================================================
  
Servo myservo; 
//Set initial position to be 90º
int pos = 90;

//=================================================================
//                              PING)))
//=================================================================
   
//Pin for signal pin in Parallax PING)))
const int pingPin = 7;

//=================================================================
//                              Motors
//=================================================================

//PWM speeds for DC motors
const int zero = 0, fullSpeed = 255, halfSpeed = 128, quarterSpeed = 64, threeQuarterSpeed = 191; 

int driveSpeed = zero;
int min_distance = 20;
bool keepScanning = true;

//=================================================================
//                              IMU
//=================================================================

Adafruit_9DOF dof = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

float roll;
float pitch;
float heading;
float accelX;
float accelY;
float accelZ; 
float gyroX;
float gyroY;
float gyroZ;

//=================================================================
//                              Extra
//=================================================================

// establish variables for duration of the ping,
// and the distance result in inches and centimeters:
long duration, inches, cm;

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

void setup() 
{
    Serial.begin(9600);
    myservo.attach(10);
    pinMode(8,OUTPUT);       //Channel A Brake Pin Initialize
    pinMode(9,OUTPUT);       //Channel B Brake Pin Initialize 
    pinMode(12,OUTPUT);      //Channel A Direction Pin Initialize
    pinMode(13,OUTPUT);      //Channel B Direction Pin Initialize

    myservo.write(90);

    initSensors();    
}

void loop() 
{

    Serial.println("===============================================");

//=================================================================
//                              SERVO
//=================================================================
    
    myservo.write(90);
    Serial.print("The position of the Servo is: ");
    Serial.println(myservo.read());
    
//=================================================================
//                              PING)))
//=================================================================
    
    duration = calculateDuration();
    cm = microsecondsToCentimeters(duration);
    Serial.print("The PING))) sensor detects object at: ");
    Serial.println(cm);

//=================================================================
//                              Motors
//=================================================================

    forward(0.1, driveSpeed);
    Serial.print("The motors are running at a speed of: ");
    Serial.println(driveSpeed);
     
//=================================================================
//                              IMU
//=================================================================

    sensors_event_t accel_event;
    sensors_event_t mag_event;
    sensors_event_t gyro_event;
    sensors_vec_t   orientation;

    accel.getEvent(&accel_event);

    accelX = accel_event.acceleration.x;
    accelY = accel_event.acceleration.y;
    accelZ = accel_event.acceleration.z;

    Serial.print("The accelerometer is reading a X, Y, Z acceleration of: ");
    Serial.print(accelX);
    Serial.print(", ");
    Serial.print(accelY);
    Serial.print(", ");
    Serial.println(accelZ);

   if (dof.accelGetOrientation(&accel_event, &orientation))
    {
        roll = orientation.roll;
        pitch = orientation.pitch;
        Serial.print("The accelerometer is reading a roll of: ");
        Serial.println(roll);
        Serial.print("The acceloremeter is reading a pitch of: ");
        Serial.println(pitch);
    }

    mag.getEvent(&mag_event);
    magX = mag_event.magnetic.x;
    magY = mag_event.magnetic.y;
    magZ = mag_event.magnetic.z;
    if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
    {
        heading = orientation.heading;
        Serial.print("The magnetometer (compass) is reading a heading of: ");
        Serial.println(heading);
    }

    gyro.getEvent(&gyro_event);
    gyroX = gyro_event.gyro.x;
    gyroY = gyro_event.gyro.y;
    gyroZ = gyro_event.gyro.z;

    Serial.print("The gyro is reading a X, Y, Z acceleration of: ");
    Serial.print(gyroX);
    Serial.print(", ");
    Serial.print(gyroY);
    Serial.print(", ");
    Serial.println(gyroZ);

//=================================================================
    Serial.println("===============================================");
    delay(1000);
  
}

//=================================================================
//                              Motors
//=================================================================

long microsecondsToInches(long microseconds) 
{
    // According to Parallax's datasheet for the PING))), there are
    // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
    // second).  This gives the distance travelled by the ping, outbound
    // and return, so we divide by 2 to get the distance of the obstacle.
    // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
    return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) 
{
    // The speed of sound is 340 m/s or 29 microseconds per centimeter.
    // The ping travels out and back, so to find the distance of the
    // object we take half of the distance travelled.
    return microseconds / 29 / 2;
}

long calculateDuration()
{
    // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
    // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
    //long duration;
    pinMode(pingPin, OUTPUT);
    digitalWrite(pingPin, LOW);
    delayMicroseconds(2);
    digitalWrite(pingPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(pingPin, LOW);
    pinMode(pingPin, INPUT);
    duration = pulseIn(pingPin, HIGH);
    return duration;
}

void forward(int duration, int driveSpeed)
{
    digitalWrite(12,LOW);
    digitalWrite(13, LOW);
    analogWrite(3, driveSpeed);
    analogWrite(11, driveSpeed); 
    delay(duration);
}

void backward(int duration, int driveSpeed)
{
    digitalWrite(12,HIGH);
    digitalWrite(13, HIGH);
    analogWrite(3, driveSpeed);
    analogWrite(11, driveSpeed);
    delay(duration);
}

void turnRight(int duration, int driveSpeed)
{
    digitalWrite(12, LOW);
    digitalWrite(13, HIGH);
    analogWrite(3, driveSpeed);
    analogWrite(11, driveSpeed);
    delay(duration);
}

void turnLeft(int duration, int driveSpeed)
{
    digitalWrite(12, HIGH);
    digitalWrite(13, LOW);
    analogWrite(3, driveSpeed);
    analogWrite(11, driveSpeed);
    delay(duration);  
}

void safetyBreakA()
{
    analogWrite(3,0);
    delay(100);
}

void safetyBreakB()
{
    analogWrite(11,0);
    delay(100);
}

void idle()
{
    digitalWrite(12,LOW);
    digitalWrite(13, LOW);
    analogWrite(3, zero);
    analogWrite(11, zero); 
}

void avoid()
{
    //keepScanning = false;
    safetyBreakA();
    safetyBreakB();
    
    backward(1000, quarterSpeed);
    
    safetyBreakA();
    safetyBreakB();
    
    turnLeft(1000, fullSpeed);
    
    safetyBreakA();
    safetyBreakB();
}
