# Mobile Robot for Navigation Tasks with Obstacle Avoidance using Potential Fields

This project implements a mobile robot capable of performing navigation tasks in an environment with obstacles using the potential fields algorithm. The robot is designed to autonomously move towards a goal location while avoiding obstacles in its path.


## Table of Contents

- [Introduction](#introduction)
- [Installation](#installation)
- [Usage](#usage)
- [Algorithm](#algorithm)
- [Demo](#demo)


## Introduction

The goal of this project is to develop a mobile robot that can navigate through an environment by employing the potential fields algorithm. The robot uses an ESP32 (programmed in Arduino IDE) as an UDP server that receives data from a PC client connected to a camera that's visualizing a workspace and sends the data necessary to detect obstacles in its surroundings. The PC client is programmed in Python. By calculating attractive and repulsive forces from the goal location and obstacles, respectively, the robot can plan its path and avoid collisions.


## Installation

1. Clone the repository:

   ```bash
   git clone https://github.com/FabianCastilloR/Mobile-Robot-for-Navigation.git
   ```

2. Install the necessary dependencies in python's PC client:

   ```bash
   pip install matplotlib
   pip install numpy
   pip install opencv-contrib-python
   ```

3. Install the necessary libraries and boards in Arduino IDE:

   ```bash
   Libraries:

   MPU6050.h from https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050

   I2Cdev.h from https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/I2Cdev

   Wifi.h from Arduino IDE Library Manager



   Boards:

   ESP32 from https://dl.espressif.com/dl/package_esp32_index.json

   You can add the ESP32 board to Arduino IDE by going to File > Preferences > Additional Boards Manager URLs and adding the link above.
   ```

4. Configure the robot hardware by connecting sensors and actuators appropriately.

   ```bash
   The robot hardware consists of the following components:

    - ESP32 microcontroller

    - MPU6050 IMU

    - 4 DC motors with incremental encoders (In this project we use 2 microgeared motors of 6VDC at 100RPM and 2 microgeared motors of 6VDC at 50RPM, both with 14 CPR encoders)

    - 2 motor drivers (In this project we use 2 VNH2SP30 motor drivers)

    - 1 LiPo battery (In this project we use a 3S 11.1V 2200mAh LiPo battery)

    - 1 5V voltage regulator (In this project we use a XL4005 voltage regulator)

    - 1 6V voltage regulator (In this project we use a LM2596 voltage regulator)
    
    - 1 camera (In this project we use a Razer Kiyo X camera)
   ```

5. Configure the robot software by setting the appropriate parameters in the PC_Client_WMultipleObstacles.py file.


## Usage

1. Launch the robot navigation python program:

   ```bash
   python PC_Client_WMultipleObstacles.py
   ```

2. Launch the robot navigation program in Arduino IDE:

   ```bash
    ESP32_Server_WMultipleObstacles.ino
    ```

3. Set the goal location for the robot manually.

4. Set the obstacles in the workspace manually.

5. Observe the robot's movement and obstacle avoidance behavior in the environment.


## Algorithm

The potential fields algorithm works by simulating forces acting on the robot. The robot experiences an attractive force towards the goal location and repulsive forces from nearby obstacles. By summing these forces and calculating the resultant direction, the robot can navigate towards the goal while avoiding obstacles.

The algorithm consists of the following steps:

1. Obtain the position of the robot, goal and obstacles from the artificial vision system to detect and measure the distance from the robot to the goal location and to the obstacles.

2. Calculate the attractive force towards the goal location based on the current position.

3. Calculate the repulsive forces from nearby obstacles based on their positions and distances.

4. Sum the attractive and repulsive forces to obtain the resultant force vector.

5. Update the robot's velocity and heading based on the resultant force.

6. Repeat the process iteratively to navigate towards the goal while avoiding obstacles.


## Demo

Demonstration videos of the robot's navigation capabilities can be found here:
    [Mobile Robot Demo 1](https://youtu.be/NOnhNL_rm1k)    
    [Mobile Robot Demo 2](https://youtube.com/shorts/4KPQuJDRXes?feature=share)