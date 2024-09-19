#pragma once

#include "Arduino.h"
#include <math.h>
//#include "Encoder.hpp"
#include "DualEncoder.hpp"
#include "EncoderOdometry.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include "MovingAverageFilter.hpp"
#include "IMU.hpp"
#include "Lidar.hpp"
//#include "OLED.hpp"

#include <Wire.h>
#include <VL6180X.h>

//Definitions

//Motor Pins
#define MOTOR_L_PWM 11
#define MOTOR_L_DIR 12
#define MOTOR_R_PWM 9
#define MOTOR_R_DIR 10

//Encoder Pins
#define EN_1_A 2 //These are the pins for the PCB encoder
#define EN_1_B 7 //These are the pins for the PCB encoder
#define EN_2_A 3 //These are the pins for the PCB encoder
#define EN_2_B 8 //These are the pins for the PCB encoder

//Lidar Pins
#define LDR_L 14
#define LDR_F 15 
#define LDR_R 16

//Lidar Address
#define LDR_AD_L 0x54
#define LDR_AD_F 0x60
#define LDR_AD_R 0x64

//Movement Commands
#define FORWARD 0
#define LEFT_ROTATE 1
#define RIGHT_ROTATE 2

// Encoder Setup
// mtrn3100::Encoder encoder_l(EN_1_A, EN_1_B);
// mtrn3100::Encoder encoder_r(EN_2_A, EN_2_B);
mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B,EN_2_A, EN_2_B);

//Motor Setup
mtrn3100::Motor motor_l(MOTOR_L_PWM,MOTOR_L_DIR);
mtrn3100::Motor motor_r(MOTOR_R_PWM,MOTOR_R_DIR);

// PID Setup
//Need to modify these pid values
mtrn3100::PIDController pid_Left_motor_foward(120, 2.0, 0.1);
mtrn3100::PIDController pid_Right_motor_foward(100, 2.0, 0.1);

mtrn3100::PIDController pid_Left_motor_rotate(120, 2.0, 0.1);
mtrn3100::PIDController pid_Right_motor_rotate(120, 2.0, 0.1);

mtrn3100::PIDController pid_Left_lidar(10, 0, 0);
mtrn3100::PIDController pid_Right_lidar(10, 0, 0);
mtrn3100::PIDController pid_Front_lidar(10, 0, 0);

mtrn3100::PIDController pid_imu(5, 0, 0);

//Sensors
MPU6050 mpu(Wire);
mtrn3100::IMU imu(mpu);
//IMU left is positive value, right is negative

mtrn3100::Lidar left_lidar(LDR_AD_L, LDR_L);
mtrn3100::Lidar front_lidar(LDR_AD_L, LDR_F);
mtrn3100::Lidar right_lidar(LDR_AD_L, LDR_R);

//Moving Average Filter
mtrn3100::MovingAverageFilter<float, 3> imu_maf;
mtrn3100::MovingAverageFilter<float, 3> left_lidar_maf;
mtrn3100::MovingAverageFilter<float, 3> right_lidar_maf;
mtrn3100::MovingAverageFilter<float, 15> pid_left_maf;
mtrn3100::MovingAverageFilter<float, 15> pid_right_maf;

//Functions
void MoveForward();
void RotateLeft();
void RotateRight();
bool ActivateMotors(int command);
void DrivePath();

void IntializeSensors();

//Variables
const float cell_length = 250;
const float wheel_radius = 17; //Remeasure
const float axle_length = 110; //Remeasure
const float rads_per_cell = cell_length / wheel_radius; //Double check
const float rads_per_rotation = (axle_length*M_PI / (4*wheel_radius)); //Double check

float pid_Left_error;
float pid_Right_error;
float pid_Left_Lidar_error;
float pid_Front_Lidar_error;
float pid_Right_Lidar_error;
float pid_IMU_error;

float left_motor_signal;
float right_motor_signal;
float current_yaw;


