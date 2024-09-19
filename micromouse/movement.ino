#include "micromouse.hpp"


float pidLeftError = 0.0;
float pidRightError = 0.0;

float maxSpeed = 100;
float maxRotationSpeed = 60;
float smallAdjustmentSpeed = 30;

float imu_straight_adj_gain = 10.0;

float initial_heading = 0.0;

bool turnComplete = true;

// Moves the MM forward HALF a cell


void moveForward(float dist) {
    // Stop the MM
    delay(100);
    motor_l.setPWM(0);
    motor_r.setPWM(0);
    delay(100);

    // Reset the encoder counts
    encoder.l_count = 0;
    encoder.r_count = 0;

    // Set PID setpoints
    pidLeft.zeroAndSetTarget(0, dist * (rads_per_cell/2));
    pidRight.zeroAndSetTarget(0, dist * (rads_per_cell/2));

    // Move the MM forward

    while (true) {
        Serial.print("Left E: ");
        Serial.println(encoder.getLeftRotation());
        Serial.print("Right E: ");
        Serial.println(encoder.getRightRotation());

        pidLeftError = pidLeft.compute(encoder.getLeftRotation());
        pidRightError = pidRight.compute(encoder.getRightRotation());

        // leftError is positive until MM reaches target
        // rightError is negative until MM reaches target

        // float diff = pidLeftError + pidRightError;

        Serial.print("Left Error: ");
        Serial.println(pidLeftError);
        Serial.print("Right Error: ");
        Serial.println(pidRightError);
        Serial.println();

        float leftSpeed = constrain(pidLeftError, -1*maxSpeed, maxSpeed);
        float rightSpeed = constrain(pidRightError, -1*maxSpeed, maxSpeed);

        leftSpeed = 0.98 * leftSpeed;

       if (fabs(leftSpeed) < 10 || fabs(rightSpeed) < 10) {
            leftSpeed = 5 * leftSpeed;
            rightSpeed = 5 * rightSpeed;
        } 

        motor_l.setPWM(leftSpeed);
        motor_r.setPWM(rightSpeed);


        if (fabs(pidLeftError) < 2 && fabs(pidRightError) < 2) {
            // print out both errors
            Serial.println("DONEE");
            Serial.print("Left Error: ");
            Serial.println(pidLeftError);
            Serial.print("Right Error: ");
            Serial.println(pidRightError);
            Serial.println();

            motor_l.setPWM(0);
            motor_r.setPWM(0);
            break;
        }

    }
    turnComplete = false;
}

void moveFowardAccurate() {
    // Stop the MM
    delay(100);
    motor_l.setPWM(0);
    motor_r.setPWM(0);

    // Reset the encoder counts
    encoder.l_count = 0;
    encoder.r_count = 0;

    // for now - reset the IMU
    if (turnComplete) {
        imu.reset();
        initial_heading = imu.getCurrentAngle();
    }

    // Set PID setpoints
    pidLeft.zeroAndSetTarget(0, rads_per_cell/2);
    pidRight.zeroAndSetTarget(0, (1*(rads_per_cell/2)));

    float heading_error = 0.0;

    // Move the MM forward

    while (true) {
        imu.update();
        // Serial.print("Left E: ");
        // Serial.println(encoder.getLeftRotation());
        // Serial.print("Right E: ");
        // Serial.println(encoder.getRightRotation());

        pidLeftError = pidLeft.compute(encoder.getLeftRotation());
        pidRightError = pidRight.compute(encoder.getRightRotation());

        // negative imu value means right heading
        // positive imu value means left heading
        heading_error = imu.getCurrentAngle() - initial_heading;
        float adjustment = heading_error * imu_straight_adj_gain;
        // if (fabs(heading_error) < 0.01 || fabs(heading_error) > 10) {
        //     adjustment = 0;
        // }

        adjustment = constrain(adjustment, -1*35, 35);

        // leftError is positive until MM reaches target
        // rightError is negative until MM reaches target

        Serial.print("Heading Error: ");
        Serial.println(heading_error);

        // float leftSpeed = constrain(pidLeftError + adjustment, -1*maxSpeed, maxSpeed);
        // float rightSpeed = constrain(pidRightError - adjustment, -1*maxSpeed, maxSpeed);

        float leftSpeed = constrain(pidLeftError, -1*maxSpeed, maxSpeed);
        float rightSpeed = constrain(pidRightError, -1*maxSpeed, maxSpeed);

        // leftSpeed = 0.98 * leftSpeed;

        leftSpeed = leftSpeed + adjustment;
        rightSpeed = rightSpeed - adjustment;

       if (fabs(leftSpeed) < 10 || fabs(rightSpeed) < 10) {
            leftSpeed = 5 * leftSpeed;
            rightSpeed = 5 * rightSpeed;
        }

        imu.update();

        motor_l.setPWM(leftSpeed);
        motor_r.setPWM(rightSpeed);


        if (fabs(pidLeftError) < 2 && fabs(pidRightError) < 2) {
            // print out both errors
            Serial.println("DONEE");
            Serial.print("Left Error: ");
            Serial.println(pidLeftError);
            Serial.print("Right Error: ");
            Serial.println(pidRightError);
            Serial.println();

            motor_l.setPWM(0);
            motor_r.setPWM(0);
            break;
        }

    }
    turnComplete = false;
}

void rotateDegreesAccurate(float degrees) {
    // Stop the MM
    delay(100);
    motor_l.setPWM(0);
    motor_r.setPWM(0);

    // Reset the encoder counts
    encoder.l_count = 0;
    encoder.r_count = 0;

    // Reset the IMU
    imu.reset();

    // Set PID setpoints
    float target_rads = degrees/90 * rads_per_rotation;
    pidLeft.zeroAndSetTarget(0, target_rads);
    pidRight.zeroAndSetTarget(0, (-1*(target_rads)));

    // Move the MM forward

    while (true) {
        imu.update();
        pidLeftError = pidLeft.compute(encoder.getLeftRotation());
        pidRightError = pidRight.compute(encoder.getRightRotation());

        // leftError is positive until MM reaches target
        // rightError is negative until MM reaches target

        float leftSpeed = constrain(pidLeftError, -1*maxRotationSpeed, maxRotationSpeed);
        float rightSpeed = constrain(pidRightError, -1*maxRotationSpeed, maxRotationSpeed);

        if (fabs(leftSpeed) < 10 || fabs(rightSpeed) < 10) {
            leftSpeed = 5 * leftSpeed;
            rightSpeed = 5 * rightSpeed;
        } 

        imu.update();

        if (fabs(pidLeftError) < 3 || fabs(pidRightError) < 3) {

            motor_l.setPWM(0);
            motor_r.setPWM(0);
            imu.update();
            delay(100);

            Serial.println("Done rotating");
            Serial.println(imu.getCurrentAngle());
            break;
        }

        motor_l.setPWM(leftSpeed);
        motor_r.setPWM(rightSpeed);
    }

    degrees = degrees * -1;

    float error = degrees - imu.getCurrentAngle();
    // while (fabs(error) > 0.015) {
    //     if (error > 0) {
    //         motor_l.setPWM(-smallAdjustmentSpeed);
    //         motor_r.setPWM(smallAdjustmentSpeed);
    //     } else {
    //         motor_l.setPWM(smallAdjustmentSpeed);
    //         motor_r.setPWM(-smallAdjustmentSpeed);
    //     }
    //     error = degrees - imu.getCurrentAngle();
    // }
    while (true) {
        if (fabs(error) < 0.005) {
            break;
        }
        if (error > 0) {
            motor_l.setPWM(-smallAdjustmentSpeed);
            motor_r.setPWM(smallAdjustmentSpeed);
        } else {
            motor_l.setPWM(smallAdjustmentSpeed);
            motor_r.setPWM(-smallAdjustmentSpeed);
        }
        error = degrees - imu.getCurrentAngle();
    }
    motor_l.setPWM(0);
    motor_r.setPWM(0);
    delay(100);
    turnComplete = true;
}

void rotateDegreesIMU(float degrees) {
    // IMU is negative on right
    // Stop the MM
    Serial.println("ROTATING using IMU");
    motor_l.setPWM(0);
    motor_r.setPWM(0);

    imu.reset();

    float error = degrees - imu.getCurrentAngle();
    Serial.print("Error: ");
    Serial.println(error);
    while (fabs(error) > 0.015) {
        if (error > 0) {
            motor_l.setPWM(-maxRotationSpeed);
            motor_r.setPWM(maxRotationSpeed);
        } else {
            motor_l.setPWM(maxRotationSpeed);
            motor_r.setPWM(-maxRotationSpeed);
        }
        error = degrees - imu.getCurrentAngle();
    }
    motor_l.setPWM(0);
    motor_r.setPWM(0);

    Serial.println("DONE rotation using the IMU");
    turnComplete = true;

}



void followPythonPath(char path[]) {
    // F is forward
    // L is 90 degrees left
    // R is 90 degrees right
    // r is 45 degrees right
    // l is 45 degrees left

    for (int i = 0; i < strlen(path); i++) {
        motor_l.setPWM(0);
        motor_r.setPWM(0);
        encoder.l_count = 0;
        encoder.r_count = 0;
        delay(100);
        if (path[i] == 'F') {
            moveForward(0.9);
        } else if (path[i] == 'f') {
            moveForward(1.414);
        }
        else if (path[i] == 'L') {
            // rotateDegrees(-87);
            rotateDegreesAccurate(-81);
        } 
        else if (path[i] == 'R') {
            // rotateDegrees(89.5);
            rotateDegreesAccurate(90.5);
        } 
        else if (path[i] == 'r') {
            // rotateDegrees(45);
            rotateDegreesAccurate(45);
        }
         else if (path[i] == 'l') {
            // rotateDegrees(-44);
            rotateDegreesAccurate(-44);
        }
    }
}

void rotateDegrees(float degrees) {
    // Stop the MM
    if (degrees < 0) {
        degrees = degrees - 2.5;
    }
    Serial.println("ROTATING");
    Serial.println(imu.getCurrentAngle());
    delay(100);
    motor_l.setPWM(0);
    motor_r.setPWM(0);
    delay(100);

    // Reset the encoder counts
    encoder.l_count = 0;
    encoder.r_count = 0;

    // Set PID setpoints
    float target_rads = degrees/90 * rads_per_rotation;
    // target_rads = target_rads < 0 ? -target_rads : target_rads;
    pidLeft.zeroAndSetTarget(0, target_rads);
    pidRight.zeroAndSetTarget(0, (-1*(target_rads)));

    // Move the MM forward

    while (true) {
        imu.update();
        // Serial.print("Left E: ");
        // Serial.println(encoder.getLeftRotation());
        // Serial.print("Right E: ");
        // Serial.println(encoder.getRightRotation());

        pidLeftError = pidLeft.compute(encoder.getLeftRotation());
        pidRightError = pidRight.compute(encoder.getRightRotation());

        // leftError is positive until MM reaches target
        // rightError is negative until MM reaches target

        // float diff = pidLeftError + pidRightError;

        // Serial.print("Left Error: ");
        // Serial.println(pidLeftError);
        // Serial.print("Right Error: ");
        // Serial.println(pidRightError);
        // Serial.println();

        float leftSpeed = constrain(pidLeftError, -1*maxRotationSpeed, maxRotationSpeed);
        float rightSpeed = constrain(pidRightError, -1*maxRotationSpeed, maxRotationSpeed);

        

        if (fabs(leftSpeed) < 10 || fabs(rightSpeed) < 10) {
            leftSpeed = 5 * leftSpeed;
            rightSpeed = 5 * rightSpeed;
        } 

        motor_l.setPWM(leftSpeed);
        motor_r.setPWM(rightSpeed);




        if (fabs(pidLeftError) < 2 && fabs(pidRightError) < 2) {
            // print out both errors
            Serial.println("DONEE");
            Serial.print("Left Error: ");
            Serial.println(pidLeftError);
            Serial.print("Right Error: ");
            Serial.println(pidRightError);
            Serial.println();

            motor_l.setPWM(0);
            motor_r.setPWM(0);

            delay(1000);

            Serial.println("Done rotating");
            Serial.println(imu.getCurrentAngle());
            break;
        }

    }
    turnComplete = true;
}


// void rotateDegrees(float degrees) {
//     // BANG BANG CONTROL
//     // Stop the MM
//     delay(100);
//     motor_l.setPWM(0);
//     motor_r.setPWM(0);

//     // Reset the encoder counts
//     encoder.l_count = 0;
//     encoder.r_count = 0;

//     float target_rads = degrees/90 * rads_per_rotation;

//     Serial.print("Target Rads: "); 
//     Serial.println(target_rads);

//     while (true) {
//         float current_rotation = encoder.getLeftRotation();
//         float error = target_rads - current_rotation;

//         // Serial.print("Current Rotation: ");
//         // Serial.println(current_rotation);

//         // Determine the direction and apply maximum control
//         if (fabs(error) > 0.005) {
//             int leftSpeed = error > 0 ? maxRotationSpeed : -maxRotationSpeed;
//             int rightSpeed = -leftSpeed;
            
//             motor_l.setPWM(leftSpeed);
//             motor_r.setPWM(rightSpeed);
//         } else {
//             // Stop the motors when the error is within the threshold
//             motor_l.setPWM(0);
//             motor_r.setPWM(0);
//             break;
//         } 
//     }
//     motor_l.setPWM(0);
//     motor_r.setPWM(0);
//     delay(100);
// }