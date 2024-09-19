#include "micromouse.hpp"

//File responsible for driving the MM
//Add functions to centre robot against wall
//Incoporate Average Filter
//

void MoveForward() {
  //Function moves MM foward 1 cell

  //Move Foward, 1 cell
  //Turn encoders enough times to reach middle of next square
  //Need to be able to read the encoders and measure the difference
  //Need to know how far the enocders need to travel
  //Need to know if there is a wall infront to stop

  //Zero encoders for starting point
  encoder.l_count = 0;
  encoder.r_count = 0;
  current_yaw =imu.getYawValue();

  //Set PIDs
  pid_Left_motor_foward.zeroAndSetTarget(encoder.getLeftRotation(), rads_per_cell);
  pid_Right_motor_foward.zeroAndSetTarget(encoder.getRightRotation(), -1*rads_per_cell);
  pid_Left_lidar.zeroAndSetTarget(0, 0);
  pid_Front_lidar.zeroAndSetTarget(0, 0);
  pid_Right_lidar.zeroAndSetTarget(0, 0);
  pid_imu.zeroAndSetTarget(current_yaw, 0);

  //Drive Motors until destination is reached, == true
  while(ActivateMotors(FORWARD) != true);

  motor_l.setPWM(0);
  motor_r.setPWM(0);

  //Reset Sensors
  delay(50);
}

void RotateLeft() {
  //Function rotates MM left 90*

  //Zero encoders for starting point
  encoder.l_count = 0;
  encoder.r_count = 0;
  current_yaw =imu.getYawValue(); 

  //Set PIDs
  pid_Left_motor_rotate.zeroAndSetTarget(encoder.getLeftRotation(), -1*rads_per_rotation);
  pid_Right_motor_rotate.zeroAndSetTarget(encoder.getRightRotation(), -1*rads_per_rotation);
  pid_Left_lidar.zeroAndSetTarget(0, 0);
  pid_Front_lidar.zeroAndSetTarget(0, 0);
  pid_Right_lidar.zeroAndSetTarget(0, 0);
  imu_maf.sample(imu.getYawValue());
  pid_imu.zeroAndSetTarget(current_yaw, 90);

  while(ActivateMotors(LEFT_ROTATE) != true);

  motor_l.setPWM(0);
  motor_r.setPWM(0);

  //Reset Sensors
  delay(50);

}

void RotateRight() {
  //Function rotates MM right 90*

  //Zero encoders for starting point
  encoder.l_count = 0;
  encoder.r_count = 0;
  current_yaw =imu.getYawValue(); 

  //Set PIDs
  pid_Left_motor_rotate.zeroAndSetTarget(encoder.getLeftRotation(), rads_per_rotation);
  pid_Right_motor_rotate.zeroAndSetTarget(encoder.getRightRotation(), rads_per_rotation);
  pid_Left_lidar.zeroAndSetTarget(0, 0);
  pid_Front_lidar.zeroAndSetTarget(0, 0);
  pid_Right_lidar.zeroAndSetTarget(0, 0);
  pid_imu.zeroAndSetTarget(current_yaw, -90);

  while(ActivateMotors(RIGHT_ROTATE) != true);

  motor_l.setPWM(0);
  motor_r.setPWM(0);

  //Reset Sensors
  delay(50);
}

bool ActivateMotors(int command) {
  //Function, receives order to either go foward, turn right or left.
  //Return value is for a loop in above function that will only exit once it has completed its task

  //Compute errors
  //Don't need lidars for Pids just checking values
  //pid_Left_Lidar_error = pid_Left_lidar.compute(left_lidar.getValue());
  //pid_Front_Lidar_error = pid_Front_lidar.compute(front_lidar.getValue());
  //pid_Right_Lidar_error = pid_Right_lidar.compute(right_lidar.getValue());
  pid_IMU_error = pid_imu.compute(imu.getYawValue());

  //Determine which movement is occuring and adjust for error
  if (command == FORWARD) {
    //Double check this value
    // if (front_lidar.getValue() < 75) {
    //   return true;
    // }

    pid_Left_error = pid_Left_motor_foward.compute(encoder.getLeftRotation());
    pid_Right_error = pid_Right_motor_foward.compute(encoder.getRightRotation());

    //If the lidar shows that the MM is in the middle of the box, reduce the IMU offset
    // if(left_lidar.getValue() < 95 && right_lidar.getValue() < 95) {
    //   pid_IMU_error = 0.5*pid_IMU_error;
    // }

    left_motor_signal = pid_Left_error; //+ pid_IMU_error;
    right_motor_signal = 0.52*pid_Right_error; //- pid_IMU_error;
    
  }
  else if (command == LEFT_ROTATE) {
      pid_Left_error = pid_Left_motor_rotate.compute(encoder.getLeftRotation());
      pid_Right_error = pid_Right_motor_rotate.compute(encoder.getRightRotation());
      //Could possibly add IMU
      left_motor_signal = pid_Left_error;
      right_motor_signal = 0.52*pid_Right_error;

  }
  else if (command == RIGHT_ROTATE) {
      pid_Left_error = pid_Left_motor_rotate.compute(encoder.getLeftRotation());
      pid_Right_error = pid_Right_motor_rotate.compute(encoder.getRightRotation());
      //Could possibly add IMU
      left_motor_signal = pid_Left_error;
      right_motor_signal = 0.56*pid_Right_error;
  }

  //Set PWM for motors
  //Set maximum speed
  if (left_motor_signal > 75) {left_motor_signal = 75;}
  if (right_motor_signal > 75) {right_motor_signal = 75;}
  if (left_motor_signal < -75) {left_motor_signal = -75;}
  if (right_motor_signal < -75) {right_motor_signal = -75;}

  //Sample signal for PID gradient
  pid_left_maf.sample(pid_Left_error);
  pid_right_maf.sample(pid_Right_error);

  //Set motor speeds
  motor_l.setPWM(left_motor_signal);
  motor_r.setPWM(right_motor_signal);

  //For Debugging
  Serial.print("Error: ");
  Serial.print(pid_Left_error);
  Serial.print("\t");
  Serial.print(pid_Right_error);
  Serial.println();
  Serial.print("Encoders: ");
  Serial.print(encoder.getLeftRotation());
  Serial.print("\t");
  Serial.print(encoder.getRightRotation());
  Serial.println();
  // Serial.print("Averages: ")
  // Serial.print(pid_left_maf.difference());
  // Serial.print("\t\t");
  // Serial.print(pid_right_maf.difference());
  // Serial.println();
  // Serial.print(pid_IMU_error);
  // Serial.print("\t\t");
  // Serial.print(imu.getYawValue());
  // Serial.println();
  
  //Check to see if movement is complete
  //Use gradient of PID
  if (abs(pid_Left_error) < 3 || abs(pid_Right_error) < 3) {
    return true;
  }
  // if (abs(pid_left_maf.average()) < 60 || abs(pid_right_maf.average()) < 60) {
  //   return true;
  // }
  else {
    return false;
  }

}

void FollowPath(char path[]) {
  //Function to follow specific commands, like drive foward 5 cells, turn left 4 times or follow given string for route

  for (int i = 0; i < sizeof(path); i++) {
    if (path[i] == 'f') {
      MoveForward();
    }
    else if (path[i] == 'l') {
      RotateLeft();
    }
    else if (path[i] == 'r') {
      RotateRight();
    }

    delay(100);
  }

}

//Other helper functions