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

### Sensors

## Software
The Raspberry Pi and the Arduino communicate through ROS using the ROSSerial package. 
### Operating System
### Communication
### Nodes
Include visualization of the nodes
### Raspberry Pi
#### Setup
##### Operating system
##### Packages installed
#### Resources used
### Nodes on the Raspberry Pi
#### IMU Filter
#### Motor Driver
#### Image Processing
#### Main Node

### Arduino
#### Arduino Node

