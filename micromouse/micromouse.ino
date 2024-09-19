#include "micromouse.hpp"

void setup() {
  Serial.begin(9600);
  //Set up sensors
  //Set up pathway
  Serial.println("Hi");
  IntializeSensors();
  Serial.println("Hello");
  delay(1000);
}

void loop() {
  // while (true) {
  //   int val = front_lidar.getValue();
  //   Serial.println(val);
  // }
  //Take in string for movement
  //FollowPath();
  //End
  MoveForward();
  RotateRight();
  MoveForward();
  RotateLeft();
  RotateLeft();
  MoveForward();
  RotateRight();
  MoveForward();
  // MoveForward();
  // MoveForward();
  // MoveForward();
  // RotateRight();
  // RotateRight();
  // RotateRight();
  // RotateRight();
  // RotateLeft();
  // RotateLeft();
  // RotateLeft();
  // RotateLeft();
  // char path[] = "frfllfrf";
  // FollowPath(path);
  //MoveForward();
  // delay(2000);
  // MoveForward();
  // delay(2000);
  Serial.println("Done");
  while(1);
 
}
