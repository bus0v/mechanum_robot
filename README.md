#This repo is part of my project for a mechanum wheeled mobile robot.![image](https://user-images.githubusercontent.com/51008991/195839631-a5430000-55e1-4173-9d1e-df261b86fe62.png)
 
The robot consists of:
  - Arduino
  - Raspberry pi
  - 4 Mechanum wheels attached to 4 12V DC motors with quadrature encoders
  - 3D printed enclosure
  - Low voltage cut off switch
  - Lipo battery
  - Motor Shield with TB6612 MOSFET drivers 
  - YDLidar X4
  - 4 ultrasound distance sensors
  
In the future I might add a Kinect onto it, to use as a depth camera
 
This repo houses the code that runs on the raspberry pi and [this](https://github.com/bus0v/Robot-Arduino) one houses the code that runs on the arduino.

The robot uses ROS as the mainframework. The Arduino consists of it's own node and communicates to the Raspberry Pi via rosserial. 

Currently these are the nodes completed:

- Kinematics: converts a raw speed vector into wheel velocities
- Odometry: recieves encoder ticks from the wheels and computes it into distance travelled and current speed. This node also broad casts the odometry information

Future work planned:
 - Implementing SLAM
 
