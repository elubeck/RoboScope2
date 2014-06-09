#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);
Adafruit_DCMotor *solenoid1 = AFMS.getMotor(3);
Adafruit_DCMotor *solenoid2 = AFMS.getMotor(4);
Adafruit_DCMotor *motor;

int pin = 0;

void setup() {
  Serial.begin(19200);
  AFMS.begin();
  motor1->setSpeed(255);
  motor2->setSpeed(255);
  solenoid1->setSpeed(255);
  solenoid2->setSpeed(255);
}

void loop() {
 while (Serial.available()) {
   int inByte_0 = Serial.read();
   int inByte_1 = Serial.read();
   switch (inByte_0) {
     case 0: motor = motor1;
     case 1: motor = motor2;
     case 2: motor = solenoid1;
     case 3: motor = solenoid2;
   }
   Serial.println(inByte_0 - 48);
   Serial.println(inByte_1);
 } 
}
