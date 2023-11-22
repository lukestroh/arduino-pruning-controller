#include <Arduino.h>

#include <Adafruit_VL53L0X.h>
#include <Adafruit_LSM9DS0.h>

// #include "avr8-stub.h"
// #include "app_api.h"
#define LOX0_ADDRESS 0x30
#define LOX1_ADDRESS 0x31

#define SHT_LOX0 7
#define SHT_LOX1 6

Adafruit_VL53L0X lox0 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(102);

// Create ToF data structs
VL53L0X_RangingMeasurementData_t tof_meas0;
VL53L0X_RangingMeasurementData_t tof_meas1;

// Output data buffer
char output_buf[128];

// Reset function
void (*reset_func) (void) = 0;

// Set up IMU
void set_up_LSM9DS0(){
  // https://github.com/adafruit/Adafruit_LSM9DS0_Library/blob/master/examples/lsm9doftest/lsm9doftest.ino
  //set up sensor
  if (!lsm.begin()) {
    Serial.println(F("Failed to boot the LSM. Rebooting in 5s..."));
    delay(5000);
    reset_func();
  }
  // Set up the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G); 

  // Set up the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS); // degrees per second
}

void set_tof_IDS() {
  // https://github.com/adafruit/Adafruit_VL53L0X/blob/master/examples/vl53l0x_dual/vl53l0x_dual.ino
  pinMode(SHT_LOX0, OUTPUT);
  pinMode(SHT_LOX1, OUTPUT);
  // Reset
  digitalWrite(SHT_LOX0, LOW);
  digitalWrite(SHT_LOX1, LOW);
  delay(10);
  digitalWrite(SHT_LOX0, HIGH);
  digitalWrite(SHT_LOX1, HIGH);
  delay(10);

  // activating LOX0, resetting LOX1
  digitalWrite(SHT_LOX0, HIGH);
  digitalWrite(SHT_LOX1, LOW);

  // init LOX0
  if (!lox0.begin(LOX0_ADDRESS)) {
    Serial.println(F("Failed to boot LOX0. Rebooting in 5s..."));
    delay(5000);
    reset_func();
  }
  delay(10);

  // Activate LOX1
  digitalWrite(SHT_LOX1, HIGH);
  delay(10);

  //init LOX1
  if (!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot LOX1. Rebooting in 5s..."));
    delay(5000);
    reset_func();
  }
  delay(10);
}

void read_tof_sensors() {
  lox0.rangingTest(&tof_meas0);
  lox1.rangingTest(&tof_meas1);
}


void read_imu_sensor() {
  lsm.readAccel();
  lsm.readGyro();
}

void create_data_buffer() {
  char data_buf[10];
  memset(output_buf, 0, sizeof(output_buf));
  strcpy(output_buf, "{tof0:");
  strcat(output_buf, dtostrf(tof_meas0.RangeMilliMeter, 6, 4, data_buf));
  strcat(output_buf, ",tof1:");
  strcat(output_buf, dtostrf(tof_meas1.RangeMilliMeter, 6, 4, data_buf));
  strcat(output_buf, ",lsm.a.x:");
  strcat(output_buf, dtostrf(lsm.accelData.x * LSM9DS0_ACCEL_MG_LSB_2G / 1000.0 * SENSORS_GRAVITY_STANDARD, 6, 4, data_buf)); // SHOULD be m/s^2
  strcat(output_buf, ",lsm.a.y:");
  strcat(output_buf, dtostrf(lsm.accelData.y * LSM9DS0_ACCEL_MG_LSB_2G / 1000.0 * SENSORS_GRAVITY_STANDARD, 6, 4, data_buf));
  strcat(output_buf, ",lsm.a.z:");
  strcat(output_buf, dtostrf(lsm.accelData.z * LSM9DS0_ACCEL_MG_LSB_2G / 1000.0 * SENSORS_GRAVITY_STANDARD, 6, 4, data_buf));
  strcat(output_buf, ",lsm.g.x:");
  strcat(output_buf, dtostrf(lsm.gyroData.x * LSM9DS0_GYRO_DPS_DIGIT_245DPS * SENSORS_DPS_TO_RADS, 6, 4, data_buf)); 
  strcat(output_buf, ",lsm.g.y:");
  strcat(output_buf, dtostrf(lsm.gyroData.y * LSM9DS0_GYRO_DPS_DIGIT_245DPS * SENSORS_DPS_TO_RADS, 6, 4, data_buf));
  strcat(output_buf, ",lsm.g.z:");
  strcat(output_buf, dtostrf(lsm.gyroData.z * LSM9DS0_GYRO_DPS_DIGIT_245DPS * SENSORS_DPS_TO_RADS, 6, 4, data_buf));
  strcat(output_buf, "}");
}

void send_data() {
  Serial.println(output_buf);
}

/*******************************************************

*******************************************************/
void setup() {
  // debug_init();

  Serial.begin(115200);

  while (!Serial) {
    delay(1);
  }
  set_tof_IDS();
  set_up_LSM9DS0();
}


void loop() {
  read_tof_sensors();
  read_imu_sensor();
  create_data_buffer();
  send_data();
}