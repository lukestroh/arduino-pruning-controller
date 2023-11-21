#include <Arduino.h>

#include <Adafruit_VL53L0X.h>
#include <Adafruit_LSM9DS0.h>

#include "avr8-stub.h"
#include "app_api.h"

Adafruit_VL53L0X lox0 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(102);

// Create ToF data structs
VL53L0X_RangingMeasurementData_t tof_meas0;
VL53L0X_RangingMeasurementData_t tof_meas1;

// Reset function
void (*reset_func) (void) = 0;

// Set up IMU
void set_up_LSM9DS0(){
  // Set up the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);

  // Set up the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
}

void set_TOF_IDS() {
   
}


/*******************************************************

*******************************************************/
void setup() {
  debug_init();

  Serial.begin(115200);

  while (!Serial) {
    delay(1);
  }
  if (!lox0.begin() || !lox1.begin()){
    Serial.println(F("Failed to boot VL53L0X. Resetting in 5s..."));
    delay(5000);
    reset_func();
  }

  lox0.startRangeContinuous();
  lox1.startRangeContinuous();

  set_up_LSM9DS0();
}


void read_tof_sensors() {
  lox0.rangingTest(&tof_meas0, false);
  if (tof_meas0.RangeStatus != 4) {
    Serial.println(tof_meas0.RangeMilliMeter);
  }
}


void loop() {
  read_tof_sensors();
}