# CRISP-Lab-land-robot
## About
Documentation for robot being developed by Mateo Guaman under Professor Usman Khan, Tufts University, 2017
## Goals
The main goal of this project is to develop a four-wheeled land robot that can detect and navigate to a certain object with specific characteristics (i.e, a certain color or shape) using computer vision.

Other secondary goals of this project are:
- Build an autonomous obstacle avoidance robot without any vision other than an ultrasonic sensor located at the front of the robot.
- Communicate with a Raspberry Pi-controlled drone being developed at the lab through ROS in real time,
- Develop a computer vision algorithm capable of detecting a specific object in real-time and keeping track of the location of said object while navigating to it.

## Challenges
One of the challenges of this project is its low-cost factor, given that the robot is only built with entry-level materials. Another challenge is working the navigation algorithm without the use of motor encoders, just by using data from the ultrasonic sensor on the front and the 9-DOF IMU which includes a 3-axis accelerometer, a 3-axis gyroscope, and a 3-axis magnetometer.

## Hardware
The robot is a four-wheeled land robot controlled by a Raspberry Pi. The Raspberry Pi controls an Arduino Mega with an Arduino Motor Shield attached to it that sends the power and direction to the DC motors on the wheels, moves the servo motor to the desired angle, and reads the data from the sensors.
### Materials
- 1 Raspberry Pi 3
- 1 Arduino Mega 2560
- 1 Arduino Motor Shield 
- 4 Sparkfun Hobby Gearmotors ROB-13302
- 4 Sparkfun Wheels-65mm ROB-13259
- 1 Parallax PING))) Ultrasonic Distance Sensor
- 1 Servo motor, Hitec HS-422
- 1 Adafruit 9-DOF IMU Breakout - L3GD20H + LSM303, ID:1714
- 1 Breadboard
- 2 24"x12" 1/8" Acrylic Sheets
- 1 LiPo 3S Battery
- 1 Anker PowerCore 13000 Battery Pack
- 1 USB wire for Arduino
- Jumper Wires
### Chassis Design
The chassis for the robot consists of a 25cm x 15cm rectangle that acts like the base plate of the robot. It also includes two side walls that go through the base plate. The DC motors are attached to these walls below the base plate. Finally, the chassis also includes a structure that holds the servo motor at the front of the robot in place. All of the acrylic parts are connected to each other with M3 screws and nuts.

The chassis was designed using Adobe Illustrator, and the file containing the design can be found on this repo inside the "chassis" folder.

### Circuit
A Fritzing file for the circuit of the robot can be found inside the "files" folder. The Arduino is powered by a LiPo 3S battery connected to the Motor Shield, and the Raspberry Pi should also be connected to power, peripherals, and to the Arduino through a USB cable. Here is an image of the circuit: 
![Circuit image](/images/CRISPRobotCircuit.png)
### Sensors
- PING))) Ultrasonic Sensor: The PING))) sensor has three pins: +5v, GND, and a signal (SIG) pin. The sensor has a range of approximately 2cm to 30 cm. To initialize the sensor, a short burst has to be sent initially, as shown in the code. 
- Adafruit 9 DOF: The Adafruit 9 DOF breakout board includes two different sensors: an L3GD20H Triple-Axis Gyro, and a LSM303 Triple-axis Accelerometer and Magnetometer. These sensors communicate through I2C/SPI. In the case of this robot, the communication is done through I2C. It should be noted that because of the physical layout of the breakout board, the accelerometer and the gyroscope are in opposite directions, which means that the readings from either of these sensors has to be multiplied by -1 to have both of the sensors in the same direction.
## Software
The Raspberry Pi and the Arduino communicate through ROS using the ROSSerial package. 
### Operating System
The Raspberry Pi runs a version of Ubuntu Mate Xenial 16.04 which included ROS Kinetic, downloaded from [german-robot.com](http://www.german-robot.com/2016/05/26/raspberry-pi-sd-card-image/). In order to setup the operating system, the instructions on the page mentioned above and on the official [ROS Kinetic Installation Guide for Ubuntu](http://wiki.ros.org/kinetic/Installation/Ubuntu) were used. 
### Communication
The method of communication between the Raspberry Pi and the Arduino, that reads all the sensors and controls the motors, is through ROS. ROS allows the data to be sent to different nodes as asynchronous messages of different types using publishers and subscribers. Additionally, it also allows data to be sent in a service/client way. One advantage of ROS is that the messages being sent are language agnostic, which means that subscribers and publishers can be coded in either C++ or Python (the two officially supported languages) but the message itself is not in any of these formats. This allows for different nodes to be programmed in either of these two language without modifying the messages sent. ROS provides many packages that can be very useful when dealing with navigation and computer vision, such as tf2 (a transform package) and support for OpenCV. Additionally, it provides different visualization tools such as rosqt_graph and rosqt_plot which can be used to display the messages being sent through each of the topics and to visualize how the nodes are interconnected. 
### ROS
The Raspberry Pi and the Arduino communicate using ROS through USB using the rosserial package. One feature of ROS is that it allows messages to be sent in specified ways. In other words, there are many preset types of messages available for better communication. For example, the ultrasonic sensor uses the sensor_msg/Range type of message to send its readings. 
### Nodes
ROS allows for there to be one main node and multiple secondary nodes that can communicate either to the main node or between secondary nodes. In the case of this robot, at the moment there is only one node on the Arduino which will in the future be one of the secondary nodes. The Arduino can only be one node, which means that all of the data is sent through the same node and that all the messages to control the motors are received on the same node. However, the goal is to have multiple nodes running on the Raspberry Pi that control different aspects of the motor controller. For example, there would be a node dedicated to processing the data from the IMU to give back an accurate representation of the position and heading of the robot, another node that sends the appropriate motor speeds and directions, and a main node that organizes everything.

TODO: Include visualization of the nodes
### Raspberry Pi
The Raspberry Pi 3 is the brains of the robot. It coordinates all of the nodes in a main node and it will also include additional nodes to do different processes such as navigation, image processing, and sensor data filtering. The Raspberry Pi is powered by a cellphone battery pack so that it can be part of the robot. 
#### Setup
As mentioned before, the Raspberry Pi is running Ubuntu Mate Xenial 16.4. The first time that the Raspberry Pi was booted, the filesystem was expanded and SSH was enabled. In the future, the Raspberry Pi will have a Raspberry Pi Camera Module V2.1 attached to it to get the images that will be used to detect objects.
##### ROS
The version of ROS installed on the Raspberry Pi is ROS Kinetic, which was setup following the instructions from the main ROS Tutorials wiki page. 
##### Packages installed
As of now, the packages installed on the Raspberry Pi are rosserial and tf2. Rosserial is mainly used to establish the communication through a serial port with the Arduino, and tf2 will be used to coordinate the transforms of the orientations and rotations of the robot. Additionally, Arduino was installed on the Raspberry Pi following the instructions from the [rosserial tutorials](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup). This installs the Arduino IDE so that the programs to the Arduino can be run directly from the IDE, although one goal is to use ROS launch to run the files once the Raspberry Pi is mounted on the robot.
#### Resources used
Some resources used to setup the Raspberry Pi to communicate with the Arduino through ROS are: 
- [Introrobotics](https://www.intorobotics.com/installing-and-setting-up-arduino-with-ros-kinetic-raspberry-pi-3/) to learn how to setup an Arduino with ROS Kinetic on a Raspberry Pi 3

- The official [rosserial wiki](http://wiki.ros.org/rosserial_arduino/Tutorials) to learn more about how rosserial works, how to create publishers and subscribers with the Arduino, examples, and more.
### Nodes on the Raspberry Pi
To come.
#### IMU Filter
#### Motor Driver
#### Image Processing
#### Main Node

### Arduino
The Arduino acts like an interface between the sensors and motors and the main processing unit of the robot, which in this case is the Raspberry Pi. The Arduino used on the robot is an Arduino Mega because of its increased processing power, memory, and number of pins available compared to an Arduino Uno. An Arduino Motor Shield controls the DC motors for the wheels, because of its ease of use and because it allows more power to be sent to the motors.  

To run files and receive data from the Arduino, a few steps need to be completed first:
1. On the terminal window, run ```roscore```
2. Upload the program to the Arduino from the Arduino IDE (the first line of this file should be ```#include <ros.h>```), making sure the right Arduino board and serial port used are selected on the "Tools window"
3. On another terminal window, run ```rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=9600``` changing ```ttyACM0``` and ```9600``` depending on the values needed.
4. After doing these, the terminal should indicate which subscribers/publishers exist, and these can be accessed using ROS tools such as ```rostopic echo {}``` with the brackets being replaced by the topic and message you want to see.
#### Arduino Node
The Arduino can only be used to run one ROS node, so this node is in charge of handling the reading of the sensors, sending them to the Raspberry Pi, and receiving commands for the motors. The code for this node can be found in this repo under the name of robotNode.ino.

##### Messages
The program includes different types of messages that are used to communicate the information to the sensors in a more organized manner. There are three different types of messages used in this program: standard messages, geometry messages, and sensor messages. 

The standard messages used in this program are:
- The standard data type Float64 is used to receive the power that should be given to the motors (0-255 to move forward or -255-0 to move backward). Floats are used in this case because the power sent to the motors can be either positive or negative to indicate in which direction the motors should rotate.
- The standard data type UInt16 is used to receive the angle that the servo motor should move to. The servo motor can only move between 0-180 degrees, which means that the unsigned integer suffices for this purpose.  

The geometry messages used in this program are:
- Vector3: This message is used to represent a three dimensional vector of float64 numbers to represent the x, y, and z axes. This message is used in this code to create 3D linear acceleration (from accelerometer), 3D angular velocity (from gyroscope), and 3D magnetic field (from magnetometer) vectors. Additionally, this type of message is also being used to send the roll, pitch, and yaw (or heading) computed on the Arduino by one of Adafruit's libraries used, described in another section of this README. 
- Vector3Stamped: This type of message is very similar to the Vector3 message, but it includes a header for the message (which is actually a ROS standard message) that includes a time stamp of the vector and a frame ID. This message is used in this program to send the rpy (roll, pitch, and yaw) vector described above to the Raspberry Pi with a timestamp so that it can be known when that message was sent. 

The sensor messages used in this program are: 
- Range: This message is used to send the range data measured by the PING))) Ultrasonic Sensor as a float32 data type in meters. Additionally, it sends information about the sensor and a header message. The information about the sensor that it sends include the radiation type that the sensor uses to detect distance (Infrared or Ultrasound), the field-of-view of the sensor as an arc in radians, and the minimum and maximum ranges of the sensor. The header message, as explained above, sends a frame ID for the sensor and a timestamp of when the reading happened.
- IMU: This message is used to send inertial information about the robot. It has the ability to send three geometry messages: an orientation quaternion, an angular velocity Vector3. and a linear acceleration Vector3. Additionally, it allows to send a 9-float64 array for each of the mentioned measurements' covariances. Finally, it also allows a header file, again, with a timestamp and a frame ID, to be sent alongside the sensor readings. In the case of this program, only the linear acceleration and the angular velocity vectors are sent, as well as the header file. The data being sent from this message allows the Raspberry Pi to know the orientation of the robot at any given time. The Adafruit 9-DOF IMU sensor also includes a magnetometer, but this type of message does not support data from this sensor to be sent. 
- MagneticField: In order to have a magnetic field reading from the IMU to be able to calculate a more accurate orientation, the MagneticField message is used to send a 3D vector of the magnetic field readings in all three axes as a Vector3 and a 9-float64 array of the magnetic field covariance. Additionally, this message also includes a header file, so it sends a timestamp and a frame ID alongside the sensor readings.

##### Using the IMU


#### Libraries Used 
To come.

![Picture of Robot](/images/robot1.JPG)
![Picture of Robot](/images/robot2.JPG)

