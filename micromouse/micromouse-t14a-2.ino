#include "micromouse.hpp"

int count = 0;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    delay(1000);

    pidLeft.zeroAndSetTarget(0, 10);
    pidRight.zeroAndSetTarget(0, 10);

    imu.begin();
    delay(1000);
    imu.update();
}

void loop() {
  // float leftSetpoint = pidLeft.compute(encoder.getLeftRotation());
  // float rightSetpoint = pidRight.compute(encoder.getRightRotation());
  // Serial.print("Left E: ");
  // Serial.println(encoder.getLeftRotation());
  // Serial.print("Right E: ");
  // Serial.println(encoder.getRightRotation());
  // Serial.println(rightSetpoint);s

  while (count == 0) {

    // moveFowardAccurate();
    // moveFowardAccurate();
    // moveFowardAccurate();
    // moveFowardAccurate();'

    // moveForward(1.0);
    // moveForward(1.0);
    // rotateDegreesAccurate(-45);
    // moveForward(1.46);
    // rotateDegreesAccurate(45);
    // rotateDegreesAccurate(45);
    // rotateDegreesAccurate(45);
    // rotateDegreesAccurate(45);

    // rotateDegreesAccurate(45);
    // rotateDegreesAccurate(45);
    // rotateDegreesAccurate(45);
    // rotateDegreesAccurate(45);

    // rotateDegreesAccurate(-90);
    // rotateDegreesAccurate(-90);
    // rotateDegreesAccurate(-90);
    // rotateDegreesAccurate(-90);

    // rotateDegreesAccurate(45);
    // rotateDegreesAccurate(45);
    // rotateDegreesAccurate(45);
    // rotateDegreesAccurate(45);

    // rotateDegreesAccurate(-45);
    // rotateDegreesAccurate(-45);
    // rotateDegreesAccurate(-45);
    // rotateDegreesAccurate(-45);

    // rotateDegreesAccurate(-45);
    // rotateDegreesAccurate(-45);
    // rotateDegreesAccurate(-45);
    // rotateDegreesAccurate(-45);
    // followPythonPath("rrrrrrrrLLLLF");
    // followPythonPath("FFFFFFFF");
    // rotateDegrees(45);
    // rotateDegrees(45);
    // rotateDegrees(45);
    // rotateDegrees(45);
    // rotateDegrees(45);
    // rotateDegrees(45);
    // rotateDegrees(45);
    // rotateDegrees(45);
    // // followPythonPath("FFFFFFFF");
    // rotateDegrees(-45);
    // rotateDegrees(-45);
    // rotateDegrees(-45);
    // rotateDegrees(-45);
    // rotateDegrees(-45);
    // rotateDegrees(-45);
    // rotateDegrees(-45);
    // rotateDegrees(-45);

    // rotateDegreesAccurate(45);
    // rotateDegreesAccurate(45);
    // rotateDegreesAccurate(45);
    // rotateDegreesAccurate(45);
    // rotateDegreesAccurate(45);
    // rotateDegreesAccurate(45);
    // rotateDegreesAccurate(45);
    // rotateDegreesAccurate(45);


    // followPythonPath("FFFFFFFFLFFLFFFFRFFFFLFFFF");
    // followPythonPath("rrrrFFRFFLFFFFFFRFFFFRFFFFLFFLFFFFRFFRFFLFFRFFRFFLFFLFFRFFRFFFFFFRFFLFFLFFRFF");
    // followPythonPath("FFFFFFFFFFFFFF");
    // followPythonPath("llllllll");
    // followPythonPath("FRFFFFLFFFLFFRFLFFFLFRFFFRFF");
    followPythonPath("FFRFFFFFFLFFRFFFRFFFFLFFLFFFFFFFFRFFFFFRFFFFFFFF");
    // followPythonPath("rrrrrrrr");

    // followPythonPath("rrrr");
    // followPythonPath("llll");


    count++;
  }

}