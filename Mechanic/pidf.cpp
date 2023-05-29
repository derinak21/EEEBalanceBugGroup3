#include "Globals.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
// Define stepper motor connections and steps per revolution:
#define dirPin 12
#define stepPin 14
#define stepsPerRevolution 200

#define dirPin2 27
#define stepPin2 26


// TUNE THESE BY TRIAL AND ERROR
const float Kp=0.5;
const float Ki=0.2;
const float Kd=0.1;

// variables for PID controller
const float tpitch=0.0;
float pepitch;
float epitch;
float ipitch;        //integral of roll and pitch
float dpitch;        //derivative of roll and pitch
float PWM;           //motor speeds


void setup() {
  // Declare pins as output:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);

  // Initialize variables
  epitch = 0.0;
  pepitch = 0.0;
  ipitch = 0.0;
  dpitch = 0.0;
}

void loop() {

  epitch = tpitch - pitch;
  ipitch += epitch;
  dpitch=g.gyro.y;
  PWM = Kp * epitch + Ki * ipitch + Kd * dpitch;
  //MAYBE DO FEEDBACK LOOP FOR PWM BECAUSE THERE WILL BE FRICTION
  if (motorSpeed1 >= 0) {
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
    motorSpeed1 = -motorSpeed1;
  }

  if (motorSpeed2 >= 0) {
    digitalWrite(dirPin2, HIGH);
  } else {
    digitalWrite(dirPin2, LOW);
    motorSpeed2 = -motorSpeed2;
  }

  for (int i = 0; i < stepsPerRevolution; i++) {
    digitalWrite(stepPin, HIGH);
    digitalWrite(stepPin2, HIGH);
    // delayMicroseconds(m1);   NOT SURE ABOUT WHERE TO IMPLEMENT THE MOTOR SPEEDS

    digitalWrite(stepPin, LOW);
    digitalWrite(stepPin2, LOW);
    // delayMicroseconds(m2);
  }

  peroll = eroll;
  pepitch = epitch;
}
