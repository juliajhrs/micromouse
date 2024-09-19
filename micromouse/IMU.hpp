#pragma once

#include "Wire.h"
#include <MPU6050_light.h>


namespace mtrn3100
{
    class IMU 
    {
        public:
          IMU(MPU6050 mpu) : imu(mpu) {
          }

          void setup() {
            byte status = imu.begin();
            while(status!=0){ } // stop everything if could not connect to MPU6050
            delay(50);
            imu.calcOffsets(); // gyro and accelero
          }
        
          float getYawValue() {
            imu.update();
            float output = imu.getAngleZ();
            currentyaw = output;
            return output;
          }

          void setOffsets() {
            imu.calcOffsets();
          }

        private:
          MPU6050 imu;
          float currentyaw;
    };
}