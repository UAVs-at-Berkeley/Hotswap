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
  
}

int startStopState = 0;

void loop() {
  // put your main code here, to run repeatedly:
  
}
