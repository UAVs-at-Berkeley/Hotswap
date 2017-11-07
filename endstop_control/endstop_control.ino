#include <AccelStepper.h>
#include <MultiStepper.h>

#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_StepperMotor *belt_stepper = AFMS.getStepper(200, 2);

//TODO: Change these
const int startStop = 1; 
const int endStop = 2;

const int resetButton = 3;

void back() { // go towards the start stop
  belt_stepper->onestep(BACKWARD, SINGLE);
}
void forward() {  // go away from start stop
  belt_stepper->onestep(FORWARD, SINGLE);
}

void release() {
  belt_stepper->release();
}

int state = 0;

int location = 0;

void setup() {
  // put your setup code here, to run once: 
  pinMode(startStop, INPUT);  
  pinMode(endStop, INPUT);
  pinMode(resetButton, INPUT);

  AFMS.begin();
  
  Serial.begin(9600);


  //reset on start
  int sts = digitalRead(startStop);
  while (!sts) {
    back();
    delay(10);
    sts = digitalRead(startStop);
  }
  release();

  location = 0;
  
  state = 0;
}

int startStopState = 0;

// ALTERNATES between going to end or start stop
void loop() {
  delay(1500);
  
  if (state == 0) {
    int est = digitalRead(endStop);
    while(!est) {
      forward();
      delay(10);
      est = digitalRead(endStop);
    }
    release();
    state = 1;
  } 
  else {

    int sts = digitalRead(startStop);
    while(!sts) {
      back();
      delay(10);
      sts = digitalRead(startStop);
    }
    release();
    state = 0;
  }
  
  delay(1500);
}
