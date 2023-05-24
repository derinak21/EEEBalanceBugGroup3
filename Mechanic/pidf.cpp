#include "Globals.h"

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

// these are the target values for pitch, roll and yaw 
const float tpitch=0.0;
const float troll=0.0;
const float tyaw=0.0;

// variables for PID controller
float peroll, pepitch;
float eroll, epitch;
float iroll, ipitch;        //integral of roll and pitch
float droll, dpitch;        //derivative of roll and pitch
int m1, m2;                 //motor speeds


void setup() {
  // Declare pins as output:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);

  // Initialize variables
  eroll = 0.0;
  epitch = 0.0;
  peroll = 0.0;
  pepitch = 0.0;
  iroll = 0.0;
  ipitch = 0.0;
  droll = 0.0;
  dpitch = 0.0;
}

void loop() {

  // roll and pitch are already variables declared in other files, still check this part
  eroll = troll - roll;
  epitch = tpitch - pitch;
  iroll += eroll;             //MIGHT BE WRONG
  ipitch += epitch;
  droll = eroll - peroll;      //MIGHT BE WRONG
  dpitch = epitch - pepitch;

  // m1 = Kp * eroll + Ki * iroll + Kd * droll;
  // m2 = Kp * epitch + Ki * ipitch + Kd * dpitch;

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
