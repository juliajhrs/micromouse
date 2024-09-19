#pragma once

#include <VL6180X.h>
#include <Wire.h>


namespace mtrn3100
{

    class Lidar
    {
    public:
      Lidar(uint8_t address, uint8_t pin) : address(address), pin(pin) {
        pinMode(pin, OUTPUT);  //Set the enable pin as output 
      }

      void setup() { 
        digitalWrite(pin, LOW);  //Write output pin low

        // Change address of sensor 1
        digitalWrite(pin, HIGH); // Enable sensor to change address
        delay(50);
        lidar_sensor.init();
        lidar_sensor.configureDefault();
        lidar_sensor.setTimeout(500);
        lidar_sensor.setAddress(address);

      }

      float getValue() {

        lidar_reading = lidar_sensor.readRangeSingleMillimeters();

        return lidar_reading;
      }


    private:
      VL6180X lidar_sensor;
      float lidar_reading = 0;
      const uint8_t address;
      const uint8_t pin;

    };
}

