#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <PID_v1.h>
#include <Wire.h>

MPU6050 mpu;

int const INTERRUPT_PIN = 2;

bool DMPReady = false;
uint8_t MPUIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint8_t FIFOBuffer[64];

Quaternion q;
VectorInt16 gy;
VectorFloat gravity;
float ypr[3];

volatile bool MPUInterrupt = false;

void DMPDataReady() {
  MPUInterrupt = true;
}

int PID_MIN_LIMIT = -255;
int PID_MAX_LIMIT = 255;
float PID_SAMPLE_TIME_IN_MILLI = 10;

float SETPOINT_PITCH_ANGLE_OFFSET = 0;
int MIN_ABSOLUTE_SPEED = 0;

double setpointPitchAngle = SETPOINT_PITCH_ANGLE_OFFSET;
double pitchGyroAngle = 0;
double pitchPIDOutput = 0;

double setpointYawRate = 0;
double yawGyroRate = 0;
double yawPIDOutput = 0;

float PID_PITCH_KP = 10;
float PID_PITCH_KI = 80;
float PID_PITCH_KD = 0.8;

float PID_YAW_KP = 0.5;
float PID_YAW_KI = 0.5;
float PID_YAW_KD = 0;

PID pitchPID(&pitchGyroAngle, &pitchPIDOutput, &setpointPitchAngle, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, DIRECT);
PID yawPID(&yawGyroRate, &yawPIDOutput, &setpointYawRate, PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, DIRECT);

int ENA = 10;
int IN1 = 5;
int IN2 = 6;
int IN3 = 7;
int IN4 = 8;
int ENB = 9;

void setupMPU() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C
#endif

  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();

  mpu.setXAccelOffset(-4018); 
  mpu.setYAccelOffset(1815); 
  mpu.setZAccelOffset(3256);   
  mpu.setXGyroOffset(109);
  mpu.setYGyroOffset(5);
  mpu.setZGyroOffset(29);  

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    MPUIntStatus = mpu.getIntStatus();
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
}

void setupMotor() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void setupPID() {
  pitchPID.SetOutputLimits(PID_MIN_LIMIT, PID_MAX_LIMIT);
  pitchPID.SetMode(AUTOMATIC);
  pitchPID.SetSampleTime(PID_SAMPLE_TIME_IN_MILLI);

  yawPID.SetOutputLimits(PID_MIN_LIMIT, PID_MAX_LIMIT);
  yawPID.SetMode(AUTOMATIC);
  yawPID.SetSampleTime(PID_SAMPLE_TIME_IN_MILLI);
}

void setup() {
  setupPID();
  setupMPU();
  setupMotor();
}

void loop() {
  if (!DMPReady) return;

  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetGyro(&gy, FIFOBuffer);

    yawGyroRate = gy.z;
    pitchGyroAngle = ypr[1] * 180 / M_PI;

    pitchPID.Compute();
    yawPID.Compute();

    Balancing(pitchPIDOutput + yawPIDOutput, pitchPIDOutput - yawPIDOutput);
  }
}

void Balancing(int speed1, int speed2) {
  if (speed1 > 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }

  if (speed2 > 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }

  speed1 = abs(speed1) + MIN_ABSOLUTE_SPEED;
  speed2 = abs(speed2) + MIN_ABSOLUTE_SPEED;

  speed1 = constrain(speed1, MIN_ABSOLUTE_SPEED, 255);
  speed2 = constrain(speed2, MIN_ABSOLUTE_SPEED, 255);

  analogWrite(ENA, speed1);
  analogWrite(ENB, speed2);
}
