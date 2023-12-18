# This repo is part of my project for a mechanum wheeled mobile robot.![image](https://user-images.githubusercontent.com/51008991/195839631-a5430000-55e1-4173-9d1e-df261b86fe62.png)
 
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
  - IMU MPU 6890
  - xbox Kinect
  

This repo houses the code that runs on the raspberry pi and [this](https://github.com/bus0v/Robot-Arduino) one houses the code that runs on the arduino.

The robot uses ROS as the main framework. The Arduino consists of it's own node and communicates to the Raspberry Pi via rosserial. 

Currently these are the nodes completed:

- Kinematics: converts a raw speed vector into wheel velocities
- Odometry: recieves encoder ticks from the wheels and computes it into distance travelled and current speed. This node also broadcasts the odometry information.

The robot launch files can also launch the lidar and teleop_twist_keyboard. Which allows the user to drive the robot around and visualize the surroundings from the laser scanner.

I also made a URDF model of the robot to load into rviz.

Future work planned:
 - Implementing SLAM
 
