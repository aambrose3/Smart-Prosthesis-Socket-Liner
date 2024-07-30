#include <Metro.h> // Include the Metro library
#include <Wire.h> // I2C Library
#include <ADS1X15.h> // Custom Library for ADCs
#include "SPSL.h" // Custom Library for the SPSL Control

//Objects
ADS1015 adc1(0x48);
ADS1015 adc2(0x49);
ADS1015 adc3(0x4A);

IntervalTimer TIMER_1;
IntervalTimer TIMER_2;

void setup() {
  Initialize();
}

void loop() {
  // Start Control
  if (start ==true){
    delay(1000);
    Start();
    start = false;
  }

  // Stop Control after 60 seconds
  if (testCount > f2*60){
    EStop();
    delay(2000);
    Vent();
    delay(2000);
    Idle();
    while(1);
  }
  // wait for new data to be measured
  if (newData == true){
    ControlUpdate();
    newData = false;
  }
}