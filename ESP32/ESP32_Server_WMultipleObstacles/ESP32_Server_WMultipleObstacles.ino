// ESP32_Server_WMultipleObstacles.ino >

/*
Authors: F. Castillo-Rodríguez, S. Almaguer-Martínez, E. Silva-Estevez & A. Saavedra-Ricalde
Date: 2023/06/06
For Instituto Tecnológico y de Estudios Superiores de Monterrey, Campus Estado de México
Project: "Mobile Robot for Navigation Tasks"

Description:

This code is the server for the mobile robot. The communication between the client and the server is through UDP protocol.
This code receives the position (x, y) in cm of the robot, the goal and the obstacles from the client and uses it to 
determine the speed of the motors through the analysis of its kinematics. This case uses the form 
qd = J^-1 * (K1 * vti + K2 * vijo) where qd is the desired speed of the motors (linear and angular), J is the Jacobian 
matrix, vti is the speed vector of the robot and vijo is the sum of the speed vectors of each obstacle. 

For the obstacle avoidance, the robot uses the potential field method, where the robot is attracted to the goal and
repelled by the obstacles. The robot also uses a PID controller to control the speed of the motors.

The robot uses the gyroscope MPU6050 to determine its yaw angle and the encoders to determine its linear speed. It also
uses the Monster Motor Shield to control the motors. 

This code works only with Python's client code "PC_Client_WMultipleObstacles.py". To take into account only one obstacle,
the user must use the server code "ESP32_Server_WOneObstacle.ino" with the client code "PC_Client_WOneObstacle.py". To skip
obstacle avoidance, the user must use the server code "ESP32_Server_WNoObstacles.ino" with the client code 
"PC_Client_WNoObstacles.py".
*/

#include <WiFiUdp.h>
#include <MPU6050.h>  // From: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
#include <I2Cdev.h>   // From: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/I2Cdev
#include <Wire.h>
#include <WiFi.h>
#include <math.h>
#include <stdlib.h>

// Front right and left motor 50 RPM (298:1 reductor)
// Back right and left motor 100 RPM (149:1 reductor)

///////////////////////////////////////////////////////// Variables /////////////////////////////////////////////////////////////////////

// Define PWM pins for motors
const int motorPWMFL = 2;   // PIN 6 FRONT MOTOR SHIELD TO PIN 2 ESP32 // GREEN
const int motorPWMFR = 4;   // PIN 5 FRONT MOTOR SHIELD TO PIN 4 ESP32 // BROWN
const int motorPWMBL = 15;  // PIN 6 BACK MOTOR SHIELD TO PIN 15 ESP32 // GREEN
const int motorPWMBR = 5;   // PIN 5 BACK MOTOR SHIELD TO PIN 5 ESP32 // BROWN

// Define direction of wheels pins
const int dirFrontMFL = 0;   // PIN 4 FRONT MOTOR SHIELD TO PIN 0 ESP32 // BLUE
const int dirBackMFL = 16;   // PIN 9 FRONT MOTOR SHIELD TO PIN 16 ESP32 // YELLOW
const int dirFrontMFR = 17;  // PIN 8 FRONT MOTOR SHIELD TO PIN 17 ESP32 // YELLOW
const int dirBackMFR = 18;   // PIN 7 FRONT MOTOR SHIELD TO PIN 18 ESP32 // BLUE
const int dirFrontMBL = 19;  // PIN 4 BACK MOTOR SHIELD TO PIN 19 ESP32 // BLUE
const int dirBackMBL = 21;   // PIN 9 BACK MOTOR SHIELD TO PIN 21 ESP32 // YELLOW
const int dirFrontMBR = 22;  // PIN 8 BACK MOTOR SHIELD TO PIN 22 ESP32 // YELLOW
const int dirBackMBR = 23;   // PIN 7 BACK MOTOR SHIELD TO PIN 23 ESP32 // BLUE

// Define encoder pins
const int encoderMFL1 = 13;  // YELLOW
const int encoderMFL2 = 14;  // GREEN
const int encoderMFR1 = 27;  // YELLOW
const int encoderMFR2 = 26;  // GREEN
const int encoderMBL1 = 35;  // YELLOW
const int encoderMBL2 = 34;  // GREEN
const int encoderMBR1 = 39;  // VN PIN; YELLOW
const int encoderMBR2 = 36;  // VP PIN; GREEN

// Variables for PWM dispatch
const int pwmFrequency = 20000;  // The frequency managed by Monster Motor Shield is 20kHz
const int pwmResolution = 8;     // With a resolution of 8-bits the value goes from 0 - 255
const int pwmChannelMFL = 0;
const int pwmChannelMFR = 2;
const int pwmChannelMBL = 1;
const int pwmChannelMBR = 3;
int pwmMFL, pwmMFR, pwmMBL, pwmMBR;

// Variables for WiFi connection
WiFiUDP udp;
const char* ssid = "Conectate y di que eres mi perra";  // Your network SSID (name)
const char* password = "soytuperra";                    // Your network password
unsigned int localUdpPort = 4210;                       // Local port to listen on
char incomingPacket[1024];                              // Buffer to hold incoming packet
char buf[255];                                          // Buffer to hold outgoing packet

// Variables for yaw detection through the gyroscope
MPU6050 accgyro;
int16_t gx, gy, gz;
double gyroYaw = 0, gyroYawRad = 0, prevdTGyro = 0;

// Variables for robot's position control
double hxRobCM = 0, hyRobCM = 0, hxGoalCM = 0, hyGoalCM = 0, hxObsCM = 0, hyObsCM = 0;
int isRobot = 0, isGoal = 0, isObstacle = 0, numObs = 0;
const int maxObs = 50;
double hxMObsCM[maxObs], hyMObsCM[maxObs], hxyObsCM[maxObs * 2], vijoMag[maxObs], posObs[maxObs][2];
char hxyObsCMChar[1024];
char* variables;

// Variables for speed adcquisition
double speedRPMMFL = 0, speedRPMMFR = 0, speedRPMMBL = 0, speedRPMMBR = 0;
double speedRadSMFL = 0, speedRadSMFR = 0, speedRadSMBL = 0, speedRadSMBR = 0;
volatile int prevEncodedMFL = 0, prevEncodedMFR = 0, prevEncodedMBL = 0, prevEncodedMBR = 0;
volatile double encoderValueMFL = 0, encoderValueMFR = 0, encoderValueMBL = 0, encoderValueMBR = 0;
double prevEncoderValueMFL = 0, prevEncoderValueMFR = 0, prevEncoderValueMBL = 0, prevEncoderValueMBR = 0;
double prevMillisMFL = 0, prevMillisMFR = 0, prevMillisMBL = 0, prevMillisMBR = 0;

// Variables for PID controller
double kpMFL = 6.50, kpMFR = 15.00, kpMBL = 6.50, kpMBR = 6.50;
double kiMFL = 0.00275, kiMFR = 0.0038, kiMBL = 0.00275, kiMBR = 0.002;
double kdMFL = 5.51, kdMFR = 6.46, kdMBL = 5.51, kdMBR = 0.20;
double desiredSpeedRPMMFL = 0, desiredSpeedRPMMFR = 0, desiredSpeedRPMMBL = 0, desiredSpeedRPMMBR = 0;
double integralMFL = 0, integralMFR = 0, integralMBL = 0, integralMBR = 0;
double prevErrorMFL = 0, prevErrorMFR = 0, prevErrorMBL = 0, prevErrorMBR = 0;
int prevPIDMFL = 0, prevPIDMFR = 0, prevPIDMBL = 0, prevPIDMBR = 0;
double prevdTMFL = 0, prevdTMFR = 0, prevdTMBL = 0, prevdTMBR = 0;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////// Program Setup /////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200);  // Begin serial communication at 115200 baud rate
  Wire.begin(25, 33);    // Begin I2C communication with pins SDA = 25 (Blue) and SCL = 33 (Yellow)

  // Enable encoder pins as inputs
  pinMode(encoderMFL1, INPUT);
  pinMode(encoderMFL2, INPUT);
  pinMode(encoderMFR1, INPUT);
  pinMode(encoderMFR2, INPUT);
  pinMode(encoderMBL1, INPUT);
  pinMode(encoderMBL2, INPUT);
  pinMode(encoderMBR1, INPUT);
  pinMode(encoderMBR2, INPUT);

  // Enable direction of the wheels pins as outputs
  pinMode(dirFrontMFL, OUTPUT);
  pinMode(dirBackMFL, OUTPUT);
  pinMode(dirFrontMFR, OUTPUT);
  pinMode(dirBackMFR, OUTPUT);
  pinMode(dirFrontMBL, OUTPUT);
  pinMode(dirBackMBL, OUTPUT);
  pinMode(dirFrontMBR, OUTPUT);
  pinMode(dirBackMBR, OUTPUT);

  // Configure LED PWM functionalitites
  ledcSetup(pwmChannelMFL, pwmFrequency, pwmResolution);
  ledcSetup(pwmChannelMFR, pwmFrequency, pwmResolution);
  ledcSetup(pwmChannelMBL, pwmFrequency, pwmResolution);
  ledcSetup(pwmChannelMBR, pwmFrequency, pwmResolution);

  // Attach the channel to the GPIO to be controlled
  ledcAttachPin(motorPWMFL, pwmChannelMFL);
  ledcAttachPin(motorPWMFR, pwmChannelMFR);
  ledcAttachPin(motorPWMBL, pwmChannelMBL);
  ledcAttachPin(motorPWMBR, pwmChannelMBR);

  // Interruptions of pulse detection of the encoders
  attachInterrupt(encoderMFL1, updateEncoderMFL, CHANGE);
  attachInterrupt(encoderMFL2, updateEncoderMFL, CHANGE);
  attachInterrupt(encoderMFR1, updateEncoderMFR, CHANGE);
  attachInterrupt(encoderMFR2, updateEncoderMFR, CHANGE);
  attachInterrupt(encoderMBL1, updateEncoderMBL, CHANGE);
  attachInterrupt(encoderMBL2, updateEncoderMBL, CHANGE);
  attachInterrupt(encoderMBR1, updateEncoderMBR, CHANGE);
  attachInterrupt(encoderMBR2, updateEncoderMBR, CHANGE);

  // Connect to WiFi
  Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);  // Connect to the network
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" CONNECTED");
  // Print local IP address and start web server
  udp.begin(localUdpPort);
  Serial.printf("UDP Server IP: %s, UDP Server Port: %d\n", WiFi.localIP().toString().c_str(), localUdpPort);

  // Initialize gyroscope
  accgyro.initialize();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////// Main Loop ////////////////////////////////////////////////////////////////////

void loop() {
  // Get the gyroscope values
  getGyroscope();
  gyroYawRad = -(gyroYaw * PI / 180);  // Convert yaw angle to radians

  // Get the speed of the wheels in RPM and rad/s with a sampling time of 50 ms
  double currentMillis = millis();
  if (currentMillis - prevMillisMFL >= 50) {
    speedRPMMFL = (-(encoderValueMFL - prevEncoderValueMFL) / ((currentMillis - prevMillisMFL) / 60000)) / (297.9238 * 14 * 2);
    speedRadSMFL = speedRPMMFL * (PI / 30);
    prevEncoderValueMFL = encoderValueMFL;
    prevMillisMFL = currentMillis;
  }
  if (currentMillis - prevMillisMFR >= 50) {
    speedRPMMFR = ((encoderValueMFR - prevEncoderValueMFR) / ((currentMillis - prevMillisMFR) / 60000)) / (297.9238 * 14 * 2);
    speedRadSMFR = speedRPMMFR * (PI / 30);
    prevEncoderValueMFR = encoderValueMFR;
    prevMillisMFR = currentMillis;
  }
  if (currentMillis - prevMillisMBL >= 50) {
    speedRPMMBL = (-(encoderValueMBL - prevEncoderValueMBL) / ((currentMillis - prevMillisMBL) / 60000)) / (297.9238 * 14);
    speedRadSMBL = speedRPMMBL * (PI / 30);
    prevEncoderValueMBL = encoderValueMBL;
    prevMillisMBL = currentMillis;
  }
  if (currentMillis - prevMillisMBR >= 50) {
    speedRPMMBR = ((encoderValueMBR - prevEncoderValueMBR) / ((currentMillis - prevMillisMBR) / 60000)) / (297.9238 * 14);
    speedRadSMBR = speedRPMMBR * (PI / 30);
    prevEncoderValueMBR = encoderValueMBR;
    prevMillisMBR = currentMillis;
  }

  // Send an empty package to the server to request the variables
  sendPackagesUDP(buf);

  // Receive the variables from the server
  variables = receivePackagesUDP();
  // Convert the variables from char to int and float to the corresponding variables
  sscanf(variables, "%d,%d,%d,%Lf,%Lf,%Lf,%Lf,%d, [%s", &isRobot, &isGoal, &isObstacle, &hxRobCM, &hyRobCM, &hxGoalCM, &hyGoalCM, &numObs, hxyObsCMChar);

  // Convert the obstacles coordinates from char to float
  getObstaclesC();

  // Get the desired speed of the wheels in RPM through its kinematic model
  getVelocityLR();

  // Determine the direction of the wheels
  wheelDirection();

  // Get the PWM of the wheels through PID control
  pwmMFL = getPID(abs(desiredSpeedRPMMFL), abs(speedRPMMFL), kpMFL, kiMFL, kdMFL, prevdTMFL, prevErrorMFL, integralMFL, prevPIDMFL);
  pwmMFR = getPID(abs(desiredSpeedRPMMFR), abs(speedRPMMFR), kpMFR, kiMFR, kdMFR, prevdTMFR, prevErrorMFR, integralMFR, prevPIDMFR);
  pwmMBL = getPID(abs(desiredSpeedRPMMBL), abs(speedRPMMBL), kpMBL, kiMBL, kdMBL, prevdTMBL, prevErrorMBL, integralMBL, prevPIDMBL);
  pwmMBR = getPID(abs(desiredSpeedRPMMBR), abs(speedRPMMBR), kpMBR, kiMBR, kdMBR, prevdTMBR, prevErrorMBR, integralMBR, prevPIDMBR);

  // Set the PWM of the wheels
  ledcWrite(pwmChannelMFL, pwmMFL);
  ledcWrite(pwmChannelMFR, pwmMFR);
  ledcWrite(pwmChannelMBL, pwmMBL);
  ledcWrite(pwmChannelMBR, pwmMBR);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////// Real Angle Obtention ///////////////////////////////////////////////////////////////

void getGyroscope() {
  // Get the gyroscope values
  accgyro.getRotation(&gx, &gy, &gz);

  // Get the time difference between the current and the previous gyroscope reading (Sampling time)
  double t = millis();
  double dTGyro = t - prevdTGyro;
  prevdTGyro = t;

  // Get the yaw angle in degrees, it is divided by 131 because the gyroscope has a sensitivity of 131 LSB/(°/s)
  gyroYaw += (gz / 131) * (dTGyro / 1000.0);

  // Keep the yaw angle between -180 and 180 degrees
  if (gyroYaw < 0) gyroYaw += 360;
  if (gyroYaw > 180) gyroYaw -= 360;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////// Encoder Pulses Detection /////////////////////////////////////////////////////////////

void updateEncoderMFL() {
  // Get the encoder values in binary
  int MSB_MFL = digitalRead(encoderMFL1);  // MSB_MFL = Most significant bit
  int LSB_MFL = digitalRead(encoderMFL2);  // LSB_MFL = Least significant bit

  // Concatenate the 2 bits to get a single number
  int encodedMFL = (MSB_MFL << 1) | LSB_MFL;

  // Adding it to the previous encoded value to find the direction of rotation
  int sumMFL = (prevEncodedMFL << 2) | encodedMFL;

  // If the direction of rotation is clockwise, the encoder value decreases, otherwise it increases
  if (sumMFL == 0b1101 || sumMFL == 0b0100 || sumMFL == 0b0010 || sumMFL == 0b1011) encoderValueMFL--;
  if (sumMFL == 0b1110 || sumMFL == 0b0111 || sumMFL == 0b0001 || sumMFL == 0b1000) encoderValueMFL++;

  // Store the current encoder value for the next iteration
  prevEncodedMFL = encodedMFL;
}

void updateEncoderMFR() {
  // Get the encoder values in binary
  int MSB_MFR = digitalRead(encoderMFR1);  // MSB_MFR = Most significant bit
  int LSB_MFR = digitalRead(encoderMFR2);  // LSB_MFR = Least significant bit

  // Concatenate the 2 bits to get a single number
  int encodedMFR = (MSB_MFR << 1) | LSB_MFR;

  // Adding it to the previous encoded value to find the direction of rotation
  int sumMFR = (prevEncodedMFR << 2) | encodedMFR;

  // If the direction of rotation is clockwise, the encoder value decreases, otherwise it increases
  if (sumMFR == 0b1101 || sumMFR == 0b0100 || sumMFR == 0b0010 || sumMFR == 0b1011) encoderValueMFR--;
  if (sumMFR == 0b1110 || sumMFR == 0b0111 || sumMFR == 0b0001 || sumMFR == 0b1000) encoderValueMFR++;

  // Store the current encoder value for the next iteration
  prevEncodedMFR = encodedMFR;
}

void updateEncoderMBL() {
  // Get the encoder values in binary
  int MSB_MBL = digitalRead(encoderMBL1);  // MSB_MBL = Most significant bit
  int LSB_MBL = digitalRead(encoderMBL2);  // LSB_MBL = Least significant bit

  // Concatenate the 2 bits to get a single number
  int encodedMBL = (MSB_MBL << 1) | LSB_MBL;

  // Adding it to the previous encoded value to find the direction of rotation
  int sumMBL = (prevEncodedMBL << 2) | encodedMBL;

  // If the direction of rotation is clockwise, the encoder value decreases, otherwise it increases
  if (sumMBL == 0b1101 || sumMBL == 0b0100 || sumMBL == 0b0010 || sumMBL == 0b1011) encoderValueMBL--;
  if (sumMBL == 0b1110 || sumMBL == 0b0111 || sumMBL == 0b0001 || sumMBL == 0b1000) encoderValueMBL++;

  // Store the current encoder value for the next iteration
  prevEncodedMBL = encodedMBL;
}

void updateEncoderMBR() {
  // Get the encoder values in binary
  int MSB_MBR = digitalRead(encoderMBR1);  // MSB_MBR = Most significant bit
  int LSB_MBR = digitalRead(encoderMBR2);  // LSB_MBR = Least significant bit

  // Concatenate the 2 bits to get a single number
  int encodedMBR = (MSB_MBR << 1) | LSB_MBR;

  // Adding it to the previous encoded value to find the direction of rotation
  int sumMBR = (prevEncodedMBR << 2) | encodedMBR;

  // If the direction of rotation is clockwise, the encoder value decreases, otherwise it increases
  if (sumMBR == 0b1101 || sumMBR == 0b0100 || sumMBR == 0b0010 || sumMBR == 0b1011) encoderValueMBR--;
  if (sumMBR == 0b1110 || sumMBR == 0b0111 || sumMBR == 0b0001 || sumMBR == 0b1000) encoderValueMBR++;

  // Store the current encoder value for the next iteration
  prevEncodedMBR = encodedMBR;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////// UDP Communication /////////////////////////////////////////////////////////////////

char* receivePackagesUDP() {
  // Determine if there is a packet available
  int packetSize = udp.parsePacket();

  // If there is a packet available
  if (packetSize) {
    // Read the packet into the buffer
    int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
    // If the size of the packet is greater than 0, add a null character at the end of the string
    if (len > 0) incomingPacket[len] = 0;
    // Return the packet
    return incomingPacket;
  }
}

void sendPackagesUDP(char* buf) {
  int i = 0;
  // Send the packet through the UDP connection where it received the packets from
  udp.beginPacket(udp.remoteIP(), udp.remotePort());
  // While the character is not a null character, send the character
  while (buf[i] != 0) udp.write((uint8_t)buf[i++]);
  // End the packet
  udp.endPacket();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////// Obstacles Centroids Obtention //////////////////////////////////////////////////////////

void getObstaclesC() {
  char bufferObs[1024];
  int bufferIndex = 0;
  int i = 0;

  // Get the obstacles centroids from the string received where the obstacles centroids are separated by a comma
  for (int j = 0; j < strlen(hxyObsCMChar); j++) {
    // If the character is not a comma, add it to the buffer
    if (hxyObsCMChar[j] != ',') {
      bufferObs[bufferIndex++] = hxyObsCMChar[j];
    }
    // If the character is a comma, add a null character at the end of the buffer and convert the buffer to a double
    else {
      bufferObs[bufferIndex] = '\0';
      hxyObsCM[i] = atof(bufferObs);
      i++;
      bufferIndex = 0;
    }
  }

  // If the buffer is not empty, add a null character at the end of the buffer and convert the buffer to a double
  if (bufferIndex > 0) {
    bufferObs[bufferIndex] = '\0';
    hxyObsCM[i] = atof(bufferObs);
  }

  // Asign the x and y coordinates of the obstacles centroids to the corresponding arrays
  for (int i = 0; i < numObs; i++) {
    hxMObsCM[i] = hxyObsCM[i];
    hyMObsCM[i] = hxyObsCM[i + numObs];
  }

  // If the number of obstacles is less than the maximum number of obstacles, set the remaining obstacles centroids to 0
  for (int i = numObs; i < maxObs; i++) {
    hxMObsCM[i] = 0;
    hyMObsCM[i] = 0;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////// Left and Right Wheels Angular and Linear Velocity Obtention ///////////////////////////////////////////

void getVelocityLR() {
  double l = 12.36, r = 2.26;  // l = Wheel Base (cm), r = Wheel Radius (cm)
  double velocityDesiredLRadS = 0, velocityDesiredRRadS = 0, velocityDesiredLRPM = 0, velocityDesiredRRPM = 0;

  // If the robot and the goal are detected, calculate the desired angular and linear velocity of the left and right wheels
  if (isRobot == 1 && isGoal == 1) {
    double a = 11;                                     // a = gain
    double vijo[2][1] = { 0, 0 };                      // vijo = vector of the sum of the obstacles repulsive values
    double posRob[2] = { hxRobCM, hyRobCM };           // posRob = position of the robot
    double posGoal[2] = { hxGoalCM, hyGoalCM };        // posGoal = position of the goal
    double K1[2][2] = { { 0.110, 0 }, { 0, 0.115 } };  // K1 = gain matrix for the attractive vector (vti)
    double K2[2][2] = { { 0, 0 }, { 0, 0 } };          // K2 = gain matrix for the repulsive vector (vijo)

    // Calculate the error between the robot and the goal
    double hError[2] = { posGoal[0] - posRob[0], posGoal[1] - posRob[1] };

    // Determine the Jacobian matrix for the rotation of the robot
    double J[2][2] = { { cos(gyroYawRad), -a * sin(gyroYawRad) }, { sin(gyroYawRad), a * cos(gyroYawRad) } };

    // If an obstacle is detected, calculate the repulsive vector (vijo)
    if (isObstacle == 1) {
      double vijoSum[2][1] = { 0, 0 };

      // Calculate the repulsive vector (vijo) for each obstacle
      for (int i = 0; i < numObs; i++) {
        posObs[i][0] = hxMObsCM[i];
        posObs[i][1] = hyMObsCM[i];

        // Calculate the euclidean distance between the robot and the obstacle
        vijoMag[i] = sqrt(pow(posRob[0] - posObs[i][0], 2) + pow(posRob[1] - posObs[i][1], 2));

        // If the euclidean distance is less than 20 cm, set the gain matrix for the repulsive vector (vijo) to -180
        if (vijoMag[i] <= 20) {
          K2[0][0] = -180;
          K2[1][1] = -180;
        } else {
          K2[0][0] = 0;
          K2[1][1] = 0;
        }

        // Calculate the repulsive vector (vijo) resulting of the sum of the repulsive vectors (vijo) of each obstacle
        vijo[0][0] = vijoSum[0][0] + (-(K2[0][0]) * (posRob[0] - posObs[i][0]) * pow(vijoMag[i], -2)) + (-(K2[0][1]) * (posRob[1] - posObs[i][1]) * pow(vijoMag[i], -2));
        vijo[1][0] = vijoSum[1][0] + (-(K2[1][0]) * (posRob[0] - posObs[i][0]) * pow(vijoMag[i], -2)) + (-(K2[1][1]) * (posRob[1] - posObs[i][1]) * pow(vijoMag[i], -2));

        vijoSum[0][0] = vijo[0][0];
        vijoSum[1][0] = vijo[1][0];
      }

      // If the number of obstacles is less than the maximum number of obstacles, set the remaining obstacles centroids to 0
      for (int i = numObs; i < maxObs; i++) {
        vijoMag[i] = 0;
        posObs[i][0] = 0;
        posObs[i][1] = 0;
      }
    }

    // Calculate the attractive vector (vti)
    double vti[2][1] = { K1[0][0] * hError[0] + K1[0][1] * hError[1], K1[1][0] * hError[0] + K1[1][1] * hError[1] };

    // Calculate the sum of the attractive vector (vti) and the repulsive vector (vijo)
    double vdi[2][1] = { vti[0][0] + vijo[0][0], vti[1][0] + vijo[1][0] };

    // Calculate the determinant of the Jacobian matrix
    double detJ = J[0][0] * J[1][1] - J[0][1] * J[1][0];

    // Calculate the inverse of the Jacobian matrix
    double inverseJ[2][2] = { { J[1][1] / detJ, -J[0][1] / detJ }, { -J[1][0] / detJ, J[0][0] / detJ } };

    // Calculate the desired angular and linear velocity of the wheels
    double qd[2] = { inverseJ[0][0] * vdi[0][0] + inverseJ[0][1] * vdi[1][0], inverseJ[1][0] * vdi[0][0] + inverseJ[1][1] * vdi[1][0] };

    double velocityDesired = qd[0];
    double angularVelocityDesired = qd[1];

    // Calculate the desired angular velocity of the right and left wheels in rad/s
    velocityDesiredLRadS = (((2 * velocityDesired) - (angularVelocityDesired * l)) / (2 * r));  // rad/s
    velocityDesiredRRadS = (((2 * velocityDesired) + (angularVelocityDesired * l)) / (2 * r));  // rad/s

    // Convert the desired angular velocity of the right and left wheels from rad/s to rpm
    velocityDesiredLRPM = velocityDesiredLRadS * 30 / PI;  // rad/s to rpm
    velocityDesiredRRPM = velocityDesiredRRadS * 30 / PI;  // rad/s to rpm

    // Limit the desired angular velocity of the right and left wheels to 48 rpm
    if (velocityDesiredLRPM > 48) velocityDesiredLRPM = 48;
    if (velocityDesiredLRPM < -48) velocityDesiredLRPM = -48;
    if (velocityDesiredRRPM > 48) velocityDesiredRRPM = 48;
    if (velocityDesiredRRPM < -48) velocityDesiredRRPM = -48;

    // As it is a differential drive robot with four wheels, both the right and left wheels have the same desired angular velocity
    desiredSpeedRPMMFL = velocityDesiredLRPM;
    desiredSpeedRPMMFR = velocityDesiredRRPM;
    desiredSpeedRPMMBL = velocityDesiredLRPM;
    desiredSpeedRPMMBR = velocityDesiredRRPM;
  }

  // If the robot or the goal is not detected, set the desired angular velocity of the right and left wheels to 0
  else {
    desiredSpeedRPMMFL = 0;
    desiredSpeedRPMMFR = 0;
    desiredSpeedRPMMBL = 0;
    desiredSpeedRPMMBR = 0;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////// Wheel Direction Control /////////////////////////////////////////////////////////////

void wheelDirection() {
  // If the desired angular velocity of the left wheels is greater than 0, set the direction of the left wheels to forward
  if (desiredSpeedRPMMFL >= 0 && desiredSpeedRPMMBL >= 0) {
    digitalWrite(dirFrontMFL, HIGH);
    digitalWrite(dirBackMFL, LOW);
    digitalWrite(dirFrontMBL, HIGH);
    digitalWrite(dirBackMBL, LOW);
  }
  // If the desired angular velocity of the left wheels is less than 0, set the direction of the left wheels to backward
  else if (desiredSpeedRPMMFL < 0 && desiredSpeedRPMMBL < 0) {
    digitalWrite(dirFrontMFL, LOW);
    digitalWrite(dirBackMFL, HIGH);
    digitalWrite(dirFrontMBL, LOW);
    digitalWrite(dirBackMBL, HIGH);
  }
  // If any other possibility occurs, set the direction of the left wheels to stop
  else {
    digitalWrite(dirFrontMFL, LOW);
    digitalWrite(dirBackMFL, LOW);
    digitalWrite(dirFrontMBL, LOW);
    digitalWrite(dirBackMBL, LOW);
  }

  // If the desired angular velocity of the right wheels is greater than 0, set the direction of the right wheels to forward
  if (desiredSpeedRPMMFR >= 0 && desiredSpeedRPMMBR >= 0) {
    digitalWrite(dirFrontMFR, HIGH);
    digitalWrite(dirBackMFR, LOW);
    digitalWrite(dirFrontMBR, HIGH);
    digitalWrite(dirBackMBR, LOW);
  }
  // If the desired angular velocity of the right wheels is less than 0, set the direction of the right wheels to backward
  else if (desiredSpeedRPMMFR < 0 && desiredSpeedRPMMBR < 0) {
    digitalWrite(dirFrontMFR, LOW);
    digitalWrite(dirBackMFR, HIGH);
    digitalWrite(dirFrontMBR, LOW);
    digitalWrite(dirBackMBR, HIGH);
  }
  // If any other possibility occurs, set the direction of the right wheels to stop
  else {
    digitalWrite(dirFrontMFR, LOW);
    digitalWrite(dirBackMFR, LOW);
    digitalWrite(dirFrontMBR, LOW);
    digitalWrite(dirBackMBR, LOW);
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////// PID Controller //////////////////////////////////////////////////////////////////

int getPID(double desiredSpeed, double encoderSpeed, double kp, double ki, double kd, double& prevdT, double& prevError, double& integral, int& prevPID) {
  // Determine the sampling time
  double t = millis();
  double dT = t - prevdT;
  prevdT = t;

  double derivative = 0;

  // Determine the error of the system
  double error = desiredSpeed - encoderSpeed;

  // Determine the integral of the system with anti-windup protection
  integral += (error * dT);
  if (integral > 255 / ki) integral = 255 / ki;
  else if (integral < -255 / ki) integral = -255 / ki;

  // Determine the derivative of the system
  if ((error - prevError) == 0) derivative = 0;
  else derivative = (error - prevError) / dT;

  // Determine the PID value limited between 0 and 255
  int pid = constrain(round((kp * error) + (ki * integral) + (kd * derivative)), 0, 255);

  // Set the previous PID value and error to the current PID value and error
  prevPID = pid;
  prevError = error;

  // Return the PID value
  return pid;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
