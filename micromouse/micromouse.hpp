#pragma once

#include "DualEncoder.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include "Wire.h"
#include "IMU.hpp"

#include <math.h>
#include <Arduino.h>

// Motor Setup
#define MOTOR_L_PWM 11
#define MOTOR_L_DIR 12
mtrn3100::Motor motor_l(MOTOR_L_PWM,MOTOR_L_DIR);

#define MOTOR_R_PWM 9
#define MOTOR_R_DIR 10
mtrn3100::Motor motor_r(MOTOR_R_PWM,MOTOR_R_DIR);

// Encoder Setup
#define EN_1_A 2 //These are the pins for the PCB encoder
#define EN_1_B 7 //These are the pins for the PCB encoder
#define EN_2_A 3 //These are the pins for the PCB encoder
#define EN_2_B 8 //These are the pins for the PCB encoder
mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B,EN_2_A, EN_2_B);

// PID Setup
mtrn3100::PIDController pidLeft(145, 2.0, 0);
mtrn3100::PIDController pidRight(150,2.0, 0);

mtrn3100::IMU imu;

// TODO - check if correct
// calculation variables
const float cell_length_mm = 250.0;
const float wheel_radius_mm = 17.0;
const float axle_length_mm = 102.5;
// const float rads_per_cell = (cell_length_mm)/(2 * M_PI * wheel_radius_mm) * 2 * M_PI;
const float rads_per_cell = cell_length_mm / wheel_radius_mm;
// const float rads_per_degree = (2 * M_PI) / 360.0;
const float rads_per_rotation = (axle_length_mm*M_PI / (4*wheel_radius_mm));

// function declarations
void moveForward(float dist);
void moveFowardAccurate();
void rotateDegrees(float degrees);
void rotateDegreesIMU(float degrees);
void rotateDegreesAccurate(float degrees);
void followPythonPath(char path[]);