// ESP32_Server_WOneObstacle.ino >

/*
Authors: F. Castillo-Rodríguez, S. Almaguer-Martínez, E. Silva-Estevez & A. Saavedra-Ricalde
Date: 2023/06/06
For Instituto Tecnológico y de Estudios Superiores de Monterrey, Campus Estado de México
Project: "Mobile Robot for Navigation Tasks"

Description:

This code is the server for the mobile robot. The communication between the client and the server is through UDP protocol.
This code receives the position (x, y) in cm of the robot, the goal and the obstacle from the client and uses it to 
determine the speed of the motors through the analysis of its kinematics. This case uses the form 
qd = J^-1 * (K1 * vti + K2 * vijo) where qd is the desired speed of the motors (linear and angular), J is the Jacobian 
matrix, vti is the speed vector of the robot and vijo is the speed vector of the obstacle.

For the obstacle avoidance, the robot uses the potential field method, where the robot is attracted to the goal and
repelled by the obstacle. The robot also uses a PID controller to control the speed of the motors.

The robot uses the gyroscope MPU6050 to determine its yaw angle and the encoders to determine its linear speed. It also
uses the Monster Motor Shield to control the motors. 

This code works only with Python's client code "PC_Client_WOneObstacle.py". To take into account multiple obstacles,
the user must use the server code "ESP32_Server_WMultipleObstacles.ino" with the client code "PC_Client_WMultipleObstacles.py".
To skip obstacle avoidance, the user must use the server code "ESP32_Server_WNoObstacles.ino" with the client code 
"PC_Client_WNoObstacles.py".
*/

#include <WiFiUdp.h>
#include <MPU6050.h>  // From: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
#include <I2Cdev.h>   // From: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/I2Cdev
#include <Wire.h>
#include <WiFi.h>
#include <math.h>

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
const char* ssid = "Conectate y di que eres mi perra";  // INFINITUME26D: QHWDbAnfm4  |  Conectate y di que eres mi perra: soytuperra  |  ABRAHAM: abra12345
const char* password = "soytuperra";
unsigned int localUdpPort = 4210;  //  port to listen on
char incomingPacket[255];          // buffer for incoming packets
char buf[255];

// Variables for yaw detection through the gyroscope
MPU6050 accgyro;
int16_t gx, gy, gz;
double gyroYaw = 0, gyroYawRad = 0, prevdTGyro = 0;

// Variables for robot's position control
double hxRobCM = 0, hyRobCM = 0, hxGoalCM = 0, hyGoalCM = 0, hxObsCM = 0, hyObsCM = 0, prevVdiAdj[2][1];
int isRobot = 0, isGoal = 0, isObstacle = 0;
char* variables;

// Variables for speed adcquisition
double speedRPMMFL = 0, speedRPMMFR = 0, speedRPMMBL = 0, speedRPMMBR = 0;
double speedRadSMFL = 0, speedRadSMFR = 0, speedRadSMBL = 0, speedRadSMBR = 0;
volatile int prevEncodedMFL = 0, prevEncodedMFR = 0, prevEncodedMBL = 0, prevEncodedMBR = 0;
volatile double encoderValueMFL = 0, encoderValueMFR = 0, encoderValueMBL = 0, encoderValueMBR = 0;
double prevEncoderValueMFL = 0, prevEncoderValueMFR = 0, prevEncoderValueMBL = 0, prevEncoderValueMBR = 0;
double prevMillisMFL = 0, prevMillisMFR = 0, prevMillisMBL = 0, prevMillisMBR = 0;

// Variables for PID controller
// double kpMFL = 0.2, kpMFR = 0.19, kpMBL = 0.12, kpMBR = 0.12;  // PI2
// double kiMFL = 0.000, kiMFR = 0.000, kiMBL = 0.0001, kiMBR = 0.0002;
// double kdMFL = 0.00, kdMFR = 0.0, kdMBL = 0.00, kdMBR = 0.001;
double kpMFL = 6.50, kpMFR = 15.00, kpMBL = 6.50, kpMBR = 6.50;  // PID
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
  Wire.begin(25, 33);    // Begin I2C communication with pins SDA = 25 and SCL = 33

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
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" CONNECTED");
  udp.begin(localUdpPort);
  Serial.printf("UDP Server IP: %s, UDP Server Port: %d\n", WiFi.localIP().toString().c_str(), localUdpPort);

  // Initialize gyroscope
  accgyro.initialize();

  prevVdiAdj[0][0] = 0, prevVdiAdj[1][0] = 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////// Main Loop ////////////////////////////////////////////////////////////////////

void loop() {
  getGyroscope();
  gyroYawRad = -(gyroYaw * PI / 180);

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

  sprintf(buf, "%Lf,%Lf,%Lf,%Lf,%Lf", speedRPMMFL, speedRPMMFR, speedRPMMBL, speedRPMMBR, gyroYawRad);
  sendPackagesUDP(buf);

  variables = receivePackagesUDP();
  sscanf(variables, "%Lf,%Lf,%Lf,%Lf,%Lf,%Lf,%d,%d,%d", &hxRobCM, &hyRobCM, &hxGoalCM, &hyGoalCM, &hxObsCM, &hyObsCM, &isRobot, &isGoal, &isObstacle);

  getVelocityLR();

  wheelDirection();

  pwmMFL = getPID(abs(desiredSpeedRPMMFL), abs(speedRPMMFL), kpMFL, kiMFL, kdMFL, prevdTMFL, prevErrorMFL, integralMFL, prevPIDMFL);
  pwmMFR = getPID(abs(desiredSpeedRPMMFR), abs(speedRPMMFR), kpMFR, kiMFR, kdMFR, prevdTMFR, prevErrorMFR, integralMFR, prevPIDMFR);
  pwmMBL = getPID(abs(desiredSpeedRPMMBL), abs(speedRPMMBL), kpMBL, kiMBL, kdMBL, prevdTMBL, prevErrorMBL, integralMBL, prevPIDMBL);
  pwmMBR = getPID(abs(desiredSpeedRPMMBR), abs(speedRPMMBR), kpMBR, kiMBR, kdMBR, prevdTMBR, prevErrorMBR, integralMBR, prevPIDMBR);

  ledcWrite(pwmChannelMFL, pwmMFL);
  ledcWrite(pwmChannelMFR, pwmMFR);
  ledcWrite(pwmChannelMBL, pwmMBL);
  ledcWrite(pwmChannelMBR, pwmMBR);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////// PID Controller //////////////////////////////////////////////////////////////////

int getPID(double desiredSpeed, double encoderSpeed, double kp, double ki, double kd, double& prevdT, double& prevError, double& integral, int& prevPID) {
  double t = millis();
  double dT = t - prevdT;
  prevdT = t;
  double derivative = 0;
  double error = desiredSpeed - encoderSpeed;
  integral += (error * dT);
  if (integral > 255 / ki) integral = 255 / ki;
  else if (integral < -255 / ki) integral = -255 / ki;
  if ((error - prevError) == 0) derivative = 0;
  else derivative = (error - prevError) / dT;
  // int pid = constrain(prevPID + round((kp * error) + (ki * integral) + (kd * derivative)), 0, 255); // PID2
  int pid = constrain(round((kp * error) + (ki * integral) + (kd * derivative)), 0, 255);  // PID
  prevPID = pid;
  prevError = error;
  return pid;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////// Wheel Direction Control /////////////////////////////////////////////////////////////

void wheelDirection() {
  if (desiredSpeedRPMMFL >= 0 && desiredSpeedRPMMBL >= 0) {
    digitalWrite(dirFrontMFL, HIGH);
    digitalWrite(dirBackMFL, LOW);
    digitalWrite(dirFrontMBL, HIGH);
    digitalWrite(dirBackMBL, LOW);
  } else if (desiredSpeedRPMMFL < 0 && desiredSpeedRPMMBL < 0) {
    digitalWrite(dirFrontMFL, LOW);
    digitalWrite(dirBackMFL, HIGH);
    digitalWrite(dirFrontMBL, LOW);
    digitalWrite(dirBackMBL, HIGH);
  } else {
    digitalWrite(dirFrontMFL, LOW);
    digitalWrite(dirBackMFL, LOW);
    digitalWrite(dirFrontMBL, LOW);
    digitalWrite(dirBackMBL, LOW);
  }

  if (desiredSpeedRPMMFR >= 0 && desiredSpeedRPMMBR >= 0) {
    digitalWrite(dirFrontMFR, HIGH);
    digitalWrite(dirBackMFR, LOW);
    digitalWrite(dirFrontMBR, HIGH);
    digitalWrite(dirBackMBR, LOW);
  } else if (desiredSpeedRPMMFR < 0 && desiredSpeedRPMMBR < 0) {
    digitalWrite(dirFrontMFR, LOW);
    digitalWrite(dirBackMFR, HIGH);
    digitalWrite(dirFrontMBR, LOW);
    digitalWrite(dirBackMBR, HIGH);
  } else {
    digitalWrite(dirFrontMFR, LOW);
    digitalWrite(dirBackMFR, LOW);
    digitalWrite(dirFrontMBR, LOW);
    digitalWrite(dirBackMBR, LOW);
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void getVelocityLR() {
  double l = 12.36, r = 2.26;  // a = gain, l = wheel base (cm), r = wheel radius (cm)
  double velocityDesiredLRadS = 0, velocityDesiredRRadS = 0, velocityDesiredLRPM = 0, velocityDesiredRRPM = 0;

  if (isRobot == 1 && isGoal == 1) {
    double a = 1.2;
    double qd[2], vijo[2][1];
    double posObs[2] = { hxObsCM, hyObsCM };
    double posRob[2] = { hxRobCM, hyRobCM };
    double posGoal[2] = { hxGoalCM, hyGoalCM };
    double K1[2][2] = { { 0.095, 0 }, { 0, 0.085 } };
    double K2[2][2] = { { -200, 0 }, { 0, -200 } };

    double hError[2] = { posGoal[0] - posRob[0], posGoal[1] - posRob[1] };
    double J[2][2] = { { cos(gyroYawRad), -a * sin(gyroYawRad) }, { sin(gyroYawRad), a * cos(gyroYawRad) } };

    if (isObstacle == 1) {
      double vijoMag = sqrt(pow(posRob[0] - posObs[0], 2) + pow(posRob[1] - posObs[1], 2));
      Serial.printf("VijoMag: %Lf \n", vijoMag);
      vijo[0][0] = (-(K2[0][0]) * (posRob[0] - posObs[0]) * pow(vijoMag, -2)) + (-(K2[0][1]) * (posRob[1] - posObs[1]) * pow(vijoMag, -2));
      vijo[1][0] = (-(K2[1][0]) * (posRob[0] - posObs[0]) * pow(vijoMag, -2)) + (-(K2[1][1]) * (posRob[1] - posObs[1]) * pow(vijoMag, -2));
    } else {
      vijo[0][0] = 0;
      vijo[1][0] = 0;
    }

    double vti[2][1] = { K1[0][0] * hError[0] + K1[0][1] * hError[1], K1[1][0] * hError[0] + K1[1][1] * hError[1] };

    // Serial.printf("Velocidad X Att: %Lf   |   Velocidad Y Att: %Lf   |   Velocidad X Rep: %Lf   |   Velocidad Y Rep: %Lf \n", vti[0][0], vti[1][0], vijo[0][0], vijo[1][0]);

    double vdi[2][1] = { vti[0][0] + vijo[0][0], vti[1][0] + vijo[1][0] };

    double detJ = J[0][0] * J[1][1] - J[0][1] * J[1][0];
    double inverseJ[2][2] = { { J[1][1] / detJ, -J[0][1] / detJ }, { -J[1][0] / detJ, J[0][0] / detJ } };

    qd[0] = inverseJ[0][0] * vdi[0][0] + inverseJ[0][1] * vdi[1][0];
    qd[1] = inverseJ[1][0] * vdi[0][0] + inverseJ[1][1] * vdi[1][0];

    double velocityDesired = qd[0];
    double angularVelocityDesired = qd[1];

    velocityDesiredLRadS = (((2 * velocityDesired) - (angularVelocityDesired * l)) / (2 * r));  // rad/s
    velocityDesiredRRadS = (((2 * velocityDesired) + (angularVelocityDesired * l)) / (2 * r));  // rad/s
    velocityDesiredLRPM = velocityDesiredLRadS * 30 / PI;                                       // rad/s to rpm
    velocityDesiredRRPM = velocityDesiredRRadS * 30 / PI;                                       // rad/s to rpm

    if (velocityDesiredLRPM > 48) velocityDesiredLRPM = 48;
    if (velocityDesiredLRPM < -48) velocityDesiredLRPM = -48;
    if (velocityDesiredRRPM > 48) velocityDesiredRRPM = 48;
    if (velocityDesiredRRPM < -48) velocityDesiredRRPM = -48;

    desiredSpeedRPMMFL = velocityDesiredLRPM;
    desiredSpeedRPMMFR = velocityDesiredRRPM;
    desiredSpeedRPMMBL = velocityDesiredLRPM;
    desiredSpeedRPMMBR = velocityDesiredRRPM;
  }

  else {
    desiredSpeedRPMMFL = 0;
    desiredSpeedRPMMFR = 0;
    desiredSpeedRPMMBL = 0;
    desiredSpeedRPMMBR = 0;
  }
}



//////////////////////////////////////////////////// Real Angle Obtention ///////////////////////////////////////////////////////////////

void getGyroscope() {
  accgyro.getRotation(&gx, &gy, &gz);
  double t = millis();
  double dTGyro = t - prevdTGyro;
  prevdTGyro = t;
  gyroYaw += (gz / 131) * (dTGyro / 1000.0);
  if (gyroYaw < 0) gyroYaw += 360;
  if (gyroYaw > 180) gyroYaw -= 360;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////// Encoder Pulses Detection /////////////////////////////////////////////////////////////

void updateEncoderMFL() {
  int MSB_MFL = digitalRead(encoderMFL1);  //MSB_MFL = most significant bit
  int LSB_MFL = digitalRead(encoderMFL2);  //LSB_MFL = least significant bit

  int encodedMFL = (MSB_MFL << 1) | LSB_MFL;        //converting the 2 pin value to single number
  int sumMFL = (prevEncodedMFL << 2) | encodedMFL;  //adding it to the previous encoded value

  if (sumMFL == 0b1101 || sumMFL == 0b0100 || sumMFL == 0b0010 || sumMFL == 0b1011) encoderValueMFL--;
  if (sumMFL == 0b1110 || sumMFL == 0b0111 || sumMFL == 0b0001 || sumMFL == 0b1000) encoderValueMFL++;

  prevEncodedMFL = encodedMFL;  //store this value for next time
}

void updateEncoderMFR() {
  int MSB_MFR = digitalRead(encoderMFR1);  //MSB_MFL = most significant bit
  int LSB_MFR = digitalRead(encoderMFR2);  //LSB_MFL = least significant bit

  int encodedMFR = (MSB_MFR << 1) | LSB_MFR;        //converting the 2 pin value to single number
  int sumMFR = (prevEncodedMFR << 2) | encodedMFR;  //adding it to the previous encoded value

  if (sumMFR == 0b1101 || sumMFR == 0b0100 || sumMFR == 0b0010 || sumMFR == 0b1011) encoderValueMFR--;
  if (sumMFR == 0b1110 || sumMFR == 0b0111 || sumMFR == 0b0001 || sumMFR == 0b1000) encoderValueMFR++;

  prevEncodedMFR = encodedMFR;  //store this value for next time
}

void updateEncoderMBL() {
  int MSB_MBL = digitalRead(encoderMBL1);  //MSB_MFL = most significant bit
  int LSB_MBL = digitalRead(encoderMBL2);  //LSB_MFL = least significant bit

  int encodedMBL = (MSB_MBL << 1) | LSB_MBL;        //converting the 2 pin value to single number
  int sumMBL = (prevEncodedMBL << 2) | encodedMBL;  //adding it to the previous encoded value

  if (sumMBL == 0b1101 || sumMBL == 0b0100 || sumMBL == 0b0010 || sumMBL == 0b1011) encoderValueMBL--;
  if (sumMBL == 0b1110 || sumMBL == 0b0111 || sumMBL == 0b0001 || sumMBL == 0b1000) encoderValueMBL++;

  prevEncodedMBL = encodedMBL;  //store this value for next time
}

void updateEncoderMBR() {
  int MSB_MBR = digitalRead(encoderMBR1);  //MSB_MFL = most significant bit
  int LSB_MBR = digitalRead(encoderMBR2);  //LSB_MFL = least significant bit

  int encodedMBR = (MSB_MBR << 1) | LSB_MBR;        //converting the 2 pin value to single number
  int sumMBR = (prevEncodedMBR << 2) | encodedMBR;  //adding it to the previous encoded value

  if (sumMBR == 0b1101 || sumMBR == 0b0100 || sumMBR == 0b0010 || sumMBR == 0b1011) encoderValueMBR--;
  if (sumMBR == 0b1110 || sumMBR == 0b0111 || sumMBR == 0b0001 || sumMBR == 0b1000) encoderValueMBR++;

  prevEncodedMBR = encodedMBR;  //store this value for next time
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////// UDP Communication /////////////////////////////////////////////////////////////////

char* receivePackagesUDP() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(incomingPacket, 255);
    if (len > 0) incomingPacket[len] = 0;
    return incomingPacket;
  }
}

void sendPackagesUDP(char* buf) {
  int i = 0;
  udp.beginPacket(udp.remoteIP(), udp.remotePort());
  while (buf[i] != 0) udp.write((uint8_t)buf[i++]);
  udp.endPacket();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////