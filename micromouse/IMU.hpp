#pragma once

# include <MPU6050_light.h>
# include "Wire.h"

namespace mtrn3100 {

class IMU {
    public:
        IMU() {
            // imu = MPU6050(Wire);
            initial_angle = 0.0;
        }

        void begin() {
            Wire.begin();
            byte status = imu.begin();
            while(status!=0){ }
            imu.calcOffsets(true,true);
            initial_angle = imu.getAngleZ();
        }

        void reset() {
            delay(200);
            // imu.calcOffsets(true,true);
            initial_angle = imu.getAngleZ();
        }

        void update() {
            imu.update();
        }

        float getCurrentAngle() {
            imu.update();
            return imu.getAngleZ() -  initial_angle;
        }

    private:
        MPU6050 imu = MPU6050(Wire);
        float initial_angle;

};

}

