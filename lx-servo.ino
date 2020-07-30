
#include "lx-servo.h"

LxServo gripper;

void setup() {
  Serial.begin(9600);
  gripper.attach(Serial2, 1);
}


void loop() {
  gripper.move(100, 10);
  delay(2000);

  gripper.grip(60, 10, 0.4, 0.2);
  delay(2000);
}
