#include "micromouse.hpp"

void IntializeSensors() {

  Wire.begin();

  imu.setup();
  left_lidar.setup();
  front_lidar.setup();
  right_lidar.setup();

}