#pragma once
#include <Arduino.h>
#include "sensors/imu.h"
#include "sensors/baro.h"
#include "control/tvc.h"
#include "flash/flightLogger.h"
#include <Servo.h>



// Hardware objects are owned by main.cpp and passed in by reference.
// No globals live here — avoids duplicate-definition linker errors.

void test_i2cScan(int type);
void test_spiScan();
void test_imuAccel(BMI323 &imu);
void test_imuGyro(BMI323 &imu);
void test_imuQuaternion(BMI323 &imu);
void test_servo(Servo &servo1, Servo &servo2);
void test_tvc(BMI323 &imu, TVC &tvc, Servo &servo1, Servo &servo2);
void test_logging(BMI323 &imu, FlightLogger &flightlogger);
void testBaro(Baro &barometer);