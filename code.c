/*
* Behnam Yosef Nezhad Arya
* Maksims Svjatoha
* KTH Degree Project in Mechatronics, First Cycle, 15 hp
* 2019-05-05
*/

#include <MPU6050_tockn.h>
#include <Wire.h>
#include <PID_v1.h>
#include <Kalman.h>
#include <LCD03.h>

MPU6050 mpu6050(Wire); //IMU

long timer = 0; //IMU timer variable

//
__________________________________________________________________________
//Declare variables
double x = 0; //IMU
double y = 0; //IMU
double ux = 0; //PID controller output x
double uy = 0; //PID controller output y
double kx = 0; //Kalman filtered angle x
double ky = 0; //Kalman filtered angle y

int writePin_y; //Will contain one of two possible values to enable y axis motor inversion
int writePin_x; //Will contain one of two possible values to enable x axis motor inversion
int SampleTime = 10; //PID sample time

//
__________________________________________________________________________
//Set up Kalman Filter

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

//
__________________________________________________________________________
//motor driver definitions, cleanup of unnecessary variables might be required

// set pin numbers:
const int LED = 13;
const int Enable_A = 3; // A low-to-high transition on the STEP input sequences the translator and advances the motor one increment
const int IN1 = 5; // Direction of rotation
const int IN2 = 6; // Mode of operation: Active/Sleep
const int Enable_B = 7; // Enable/Disable the Driver operation
const int IN3 = 9; // Reset when active turns off all of the FET outputs
const int IN4 = 10; // Microstep Select
const int Threshold = 20;

#define IN1_L digitalWrite(IN1, LOW);
#define IN1_H digitalWrite(IN1, HIGH);
#define IN2_L digitalWrite(IN2, LOW);
#define IN2_H digitalWrite(IN2, HIGH);
#define IN3_L digitalWrite(IN3, LOW);
#define IN3_H digitalWrite(IN3, HIGH);
#define IN4_L digitalWrite(IN4, LOW);
#define IN4_H digitalWrite(IN4, HIGH);

//motor definitions, end
//

__________________________________________________________________________
//PID definitions, start


//Define Variables we’ll be connecting to
double Setpointy, Inputy, Outputy;
double Setpointx, Inputx, Outputx;

//Define the aggressive and conservative Tuning Parameters
double consKpy=12.81, consKiy=17.50, consKdy=0.1; //Last known good configuration: 15, 15, 0.005
double consKpx=32.5, consKix=12.50, consKdx=0.1; //Last known good configuration: 38.125, 10.00, 0.005

//Specify the links and initial tuning parameters
PID PIDy(&Inputy, &Outputy, &Setpointy, consKpy, consKiy, consKdy, DIRECT);
PID PIDx(&Inputx, &Outputx, &Setpointx, consKpx, consKix, consKdx, DIRECT);

//PID definitions, end
//
__________________________________________________________________________

void setup() {
Serial.begin(9600); //IMU
Wire.begin(); //IMU
mpu6050.begin(); //IMU
mpu6050.calcGyroOffsets(true); //IMU

// set the digital pin as output: //Motor driver
pinMode(LED, OUTPUT); //Motor driver
pinMode(Enable_A, OUTPUT); //Motor driver
pinMode(Enable_B, OUTPUT); //Motor driver
pinMode(IN1, OUTPUT); //Motor driver
pinMode(IN2, OUTPUT); //Motor driver
pinMode(IN3, OUTPUT); //Motor driver
pinMode(IN4, OUTPUT); //Motor driver

//Set the state
digitalWrite(LED, LOW); //Motor driver
digitalWrite(Enable_A, HIGH); //Motor driver
digitalWrite(Enable_B, HIGH); //Motor driver
digitalWrite(IN1, LOW); //Motor driver
digitalWrite(IN2, LOW); //Motor driver
digitalWrite(IN3, LOW); //Motor driver
digitalWrite(IN4, LOW); //Motor driver

//initialize the variables we’re linked to
Inputy = ky; //PID
Inputx = kx; //PID
Setpointy = 0; //PID
Setpointx = 0; //PID

//turn the PID on //PID
PIDy.SetMode(AUTOMATIC); //PID
PIDx.SetMode(AUTOMATIC); //PID

//Changes output to include negative values (standard is 0-255).
PIDy.SetOutputLimits(-255,255); //PID
PIDx.SetOutputLimits(-255,255); //PID

x = mpu6050.getGyroAngleX(); //IMU
y = mpu6050.getGyroAngleY(); //IMU

kalmanX.setAngle(x); // Set starting angle
kalmanY.setAngle(y);

PIDy.SetSampleTime(SampleTime);
PIDx.SetSampleTime(SampleTime);

}

void loop() {

//Reading the sensor data (gyroscope data around x and y axis).

mpu6050.update();

x = mpu6050.getGyroAngleX(); //IMU
y = mpu6050.getGyroAngleY(); //IMU

//Applying a Kalman filter to raw signal data.

double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
timer = micros();

double gyroXrate = x / 131.0; // Convert to deg/s
double gyroYrate = y / 131.0; // Convert to deg/s

kx = kalmanX.getAngle(x, gyroXrate, dt); //Kalman filtered gyro signal, x axis.
ky = kalmanY.getAngle(y, gyroYrate, dt); //Kalman filtered gyro signal, y axis.

//PID tuning. Possible to implement different K values for large angular deviations.

PIDy.SetTunings(consKpy, consKiy, consKdy);
PIDx.SetTunings(consKpx, consKix, consKdx);

//The PID computation:

Inputy = ky;
Inputx = kx;

PIDy.Compute(); //Comment out to turn off Y axis stabilization.
PIDx.Compute(); //Comment out to turn off X axis stabilization.

uy = Outputy;
ux = Outputx;

//Handling of negative values. To reverse motor polarity, the voltage must be applied on a different pin.

if (uy < 0) {writePin_y = 10;}
if (uy > 0) {writePin_y = 9;}
if (ux < 0) {writePin_x = 6;}
if (ux > 0) {writePin_x = 5;}

//Only positive bit values between 0 and 255 are accepted. Once the pin is chosen, the bit signal must be positive.

if (uy < 0) {uy = -uy;}
if (ux < 0) {ux = -ux;}

analogWrite(writePin_y, uy);
analogWrite(writePin_x, ux);

Serial.print("x : ");Serial.print(x);Serial.print(", "); //IMU
Serial.print("kx : ");Serial.print(kx);Serial.print(", "); //IMU
Serial.print("y : ");Serial.print(y);Serial.print(", "); //IMU
Serial.print("ky : ");Serial.print(ky);Serial.print(", "); //IMU
Serial.print("ux : ");Serial.print(ux);Serial.print(", "); //IMU
Serial.print("uy : ");Serial.print(uy);Serial.print(", "); //IMU
Serial.print(’\n’); //IMU

}